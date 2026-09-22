#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo_mpc.yaml | yaml配置文件路径}"
  "{start-index s  | 0                 | 视频起始帧下标}"
  "{end-index e    | 0                 | 视频结束帧下标，0表示到视频结束}"
  "{view v         | 1                 | 是否显示图像窗口，0为无窗口离线跑（无X显示时用）}"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto view = cli.get<int>("view") != 0;

  tools::Plotter plotter;
  tools::Exiter exiter;

  cv::VideoCapture video(fmt::format("{}.avi", input_path));
  std::ifstream text(fmt::format("{}.txt", input_path));
  if (!video.isOpened() || !text) {
    tools::logger()->error("Failed to open demo input: {}", input_path);
    return 1;
  }

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;
    if (!video.read(img) || img.empty()) break;

    double t, w, x, y, z;
    if (!(text >> t >> w >> x >> y >> z)) break;
    auto timestamp = t0 + std::chrono::microseconds(static_cast<int>(t * 1e6));
    solver.set_R_gimbal2world(Eigen::Quaterniond(w, x, y, z));

    auto armors = yolo.detect(img, frame_count);
    auto targets = tracker.track(armors, timestamp);
    // 必须传入本帧的（视频）时间戳：回放时视频时间与墙钟无关，
    // 用默认墙钟会算出 -数百秒的 dt，直接把 EKF 状态打飞
    auto plan = planner.plan(
      targets.empty() ? std::optional<auto_aim::Target>{} : std::optional<auto_aim::Target>{targets.front()},
      22, timestamp);

    nlohmann::json data;
    data["frame"] = frame_count;
    data["t"] = t;
    data["gimbal_yaw"] = tools::eulers(Eigen::Quaterniond(w, x, y, z), 2, 1, 0)[0];
    data["target_yaw"] = plan.target_yaw;
    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;
    if (!targets.empty()) data["target_w"] = targets.front().ekf_x()[7];
    plotter.plot(data);

    if (!targets.empty()) {
      auto target = targets.front();
      auto state = target.ekf_x();
      // 每帧目标状态：r/l 是否稳定是 tracker 会不会把目标判丢的直接指标
      tools::logger()->debug(
        "[Demo] frame={} last_id={} w={:.3f} r={:.3f} l={:.3f} angle={:.1f} center=({:.2f},{:.2f},{:.2f})",
        frame_count, target.last_id, state[7], state[8], state[9], state[6] * 57.3,
        state[0], state[2], state[4]);

      if (view) {
        for (const auto & xyza : target.armor_xyza_list()) {
          auto points = solver.reproject_armor(
            xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, points, {0, 255, 0});
        }

        if (plan.control) {
          auto points = solver.reproject_armor(
            planner.debug_xyza.head(3), planner.debug_xyza[3], target.armor_type, target.name);
          tools::draw_points(img, points, {0, 0, 255});
        }

        tools::draw_text(
          img,
          fmt::format("w: {:.2f}  r: {:.2f}  a: {:.1f}deg", state[7], state[8], state[6] * 57.3),
          {10, 35}, {255, 255, 255});
      }
    }

    if (view) {
      tools::draw_text(
        img,
        fmt::format(
          "target yaw: {:.1f}  plan yaw: {:.1f}  vel: {:.2f}  acc: {:.2f}",
          plan.target_yaw * 57.3, plan.yaw * 57.3, plan.yaw_vel, plan.yaw_acc),
        {10, 65}, {0, 255, 255});
      tools::draw_text(
        img, "green: predicted armors  red: MPC aim armor", {10, 95}, {0, 255, 255});

      cv::resize(img, img, {}, 0.5, 0.5);
      cv::imshow("mpc_reprojection", img);
      auto key = cv::waitKey(30);
      if (key == 'q') break;
    }
  }

  return 0;
}