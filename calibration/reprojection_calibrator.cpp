#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

namespace
{
const std::string keys =
  "{help h usage ? | | show help}"
  "{config-path c | configs/demo_mpc.yaml | yaml config}"
  "{@input-path | records/2026-09-15_09-59-23 | video/timestamp base path}";

struct Extrinsic
{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

Extrinsic load_extrinsic(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto rotation_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto translation_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  return {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(rotation_data.data()),
    Eigen::Vector3d(translation_data.data())};
}

void print_extrinsic(const Extrinsic & extrinsic)
{
  std::cout << std::setprecision(12)
            << "R_camera2gimbal: [";
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (row != 0 || col != 0) std::cout << ", ";
      std::cout << extrinsic.rotation(row, col);
    }
  }
  std::cout << "]\nt_camera2gimbal: ["
            << extrinsic.translation.x() << ", "
            << extrinsic.translation.y() << ", "
            << extrinsic.translation.z() << "]\n";
}

void draw_model(
  cv::Mat & image, const auto_aim::Target & target, auto_aim::Solver & solver)
{
  for (const auto & xyza : target.armor_xyza_list()) {
    auto points = solver.reproject_armor(
      xyza.head(3), xyza[3], target.armor_type, target.name);
    tools::draw_points(image, points, {0, 255, 0}, 2);
  }
}

void print_help()
{
  std::cout << "Keys: a/d yaw, w/s pitch, q/e roll, u/j tx, h/k ty, n/m tz, "
               "p print, space next frame, esc quit\n"
            << "Rotation step: 0.1 deg, translation step: 1 mm; starts frozen\n";
}

void draw_status(cv::Mat & image, const Extrinsic & current, const Extrinsic & initial)
{
  auto delta_rotation = current.rotation * initial.rotation.transpose();
  auto delta_ypr = tools::eulers(delta_rotation, 2, 1, 0) * 180.0 / CV_PI;
  auto delta_translation = current.translation - initial.translation;
  tools::draw_text(
    image,
    fmt::format(
      "dYaw {:.2f}  dPitch {:.2f}  dRoll {:.2f} deg",
      delta_ypr[0], delta_ypr[1], delta_ypr[2]),
    {10, 30}, {0, 255, 255});
  tools::draw_text(
    image,
    fmt::format(
      "dT: x {:.1f}  y {:.1f}  z {:.1f} mm | SPACE next frame",
      delta_translation.x() * 1000.0, delta_translation.y() * 1000.0,
      delta_translation.z() * 1000.0),
    {10, 60}, {0, 255, 255});
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    print_help();
    return 0;
  }

  const auto config_path = cli.get<std::string>("config-path");
  const auto input_path = cli.get<std::string>(0);
  cv::VideoCapture video(fmt::format("{}.avi", input_path));
  std::ifstream timestamp_file(fmt::format("{}.txt", input_path));
  if (!video.isOpened() || !timestamp_file) {
    std::cerr << "Failed to open input: " << input_path << "\n";
    return 1;
  }

  auto extrinsic = load_extrinsic(config_path);
  const auto initial_extrinsic = extrinsic;
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  print_help();
  print_extrinsic(extrinsic);

  bool advance_frame = true;
  int frame_count = 0;
  cv::Mat image;
  std::list<auto_aim::Armor> armors;
  std::list<auto_aim::Target> targets;
  double timestamp, qw, qx, qy, qz;
  while (true) {
    if (advance_frame) {
      if (!video.read(image) || image.empty()) break;
      if (!(timestamp_file >> timestamp >> qw >> qx >> qy >> qz)) break;
      solver.set_R_gimbal2world(Eigen::Quaterniond(qw, qx, qy, qz));
      armors = yolo.detect(image, frame_count);
      targets = tracker.track(armors, std::chrono::steady_clock::now());
      advance_frame = false;
    }

    solver.set_camera_extrinsic(extrinsic.rotation, extrinsic.translation);
    cv::Mat display = image.clone();
    for (const auto & armor : armors) {
      tools::draw_points(display, armor.points, {255, 0, 0}, 2);
    }
    if (!targets.empty()) draw_model(display, targets.front(), solver);
    draw_status(display, extrinsic, initial_extrinsic);
    tools::draw_text(
      display, "blue=detected green=model | SPACE next | p print | ESC quit", {10, 90}, {0, 255, 255});
    cv::imshow("reprojection_calibrator", display);

    int key = cv::waitKey(0);
    if (key == 27 || key == 'x') break;
    if (key == ' ') { advance_frame = true; frame_count++; continue; }

    constexpr double angle_step = 0.1 * CV_PI / 180.0;
    constexpr double translation_step = 0.001;
    if (key == 'a') extrinsic.rotation = Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitZ()) * extrinsic.rotation;
    if (key == 'd') extrinsic.rotation = Eigen::AngleAxisd(-angle_step, Eigen::Vector3d::UnitZ()) * extrinsic.rotation;
    if (key == 'w') extrinsic.rotation = Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitY()) * extrinsic.rotation;
    if (key == 's') extrinsic.rotation = Eigen::AngleAxisd(-angle_step, Eigen::Vector3d::UnitY()) * extrinsic.rotation;
    if (key == 'q') extrinsic.rotation = Eigen::AngleAxisd(angle_step, Eigen::Vector3d::UnitX()) * extrinsic.rotation;
    if (key == 'e') extrinsic.rotation = Eigen::AngleAxisd(-angle_step, Eigen::Vector3d::UnitX()) * extrinsic.rotation;
    if (key == 'u') extrinsic.translation.x() += translation_step;
    if (key == 'j') extrinsic.translation.x() -= translation_step;
    if (key == 'h') extrinsic.translation.y() += translation_step;
    if (key == 'k') extrinsic.translation.y() -= translation_step;
    if (key == 'n') extrinsic.translation.z() += translation_step;
    if (key == 'm') extrinsic.translation.z() -= translation_step;
    if (key == 'p') print_extrinsic(extrinsic);
  }

  print_extrinsic(extrinsic);
  return 0;
}
// a / d   yaw ±0.1°  装甲板应该尽量正对相机
// w / s   pitch ±0.1°
// q / e   roll ±0.1°
// u / j   tx ±1 mm t矩阵一般物理测定得到不需要手动调整
// h / k   ty ±1 mm
// n / m   tz ±1 mm
// 空格    暂停/继续
// p       终端输出当前矩阵
// Esc/x   退出