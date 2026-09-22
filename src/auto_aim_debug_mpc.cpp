#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{record r        | false                 | 是否录制图像和IMU数据}";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto record = cli.get<bool>("record");

  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);
  auto_aim::Plan latest_plan;
  std::mutex latest_plan_mutex;

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;
    int counter = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);
      {
        std::lock_guard<std::mutex> lock(latest_plan_mutex);
        latest_plan = plan;
      }
      // // 临时屏蔽弹道补偿：使用极大子弹速度（近似直线）
      // double fake_bullet_speed = 2200.0; // 单位 m/s，足够大即可
      // auto plan = planner.plan(target, fake_bullet_speed);
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, -plan.pitch, -plan.pitch_vel,
        -plan.pitch_acc); //去掉负号？

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;
      if (counter % 15 == 0) { // 每10次循环打印一次，避免输出过快
        std::cout << "\n===== Gimbal and Planning Data =====\n";
        std::cout << "Gimbal Yaw: " << std::fixed << std::setprecision(4) << gs.yaw << " rad\n";
        std::cout << "Gimbal Yaw Velocity: " << gs.yaw_vel << " rad/s\n";
        std::cout << "Gimbal Pitch: " << gs.pitch << " rad\n";
        std::cout << "Gimbal Pitch Velocity: " << gs.pitch_vel << " rad/s\n";
        
        std::cout << "\nTarget Yaw: " << plan.target_yaw << " rad\n";
        std::cout << "Target Pitch: " << plan.target_pitch << " rad\n";
        
        std::cout << "\nPlanned Yaw: " << plan.yaw << " rad\n";
        std::cout << "Planned Yaw Velocity: " << plan.yaw_vel << " rad/s\n";
        std::cout << "Planned Yaw Acceleration: " << plan.yaw_acc << " rad/s²\n";
        
        std::cout << "\nPlanned Pitch: " << plan.pitch << " rad\n";
        std::cout << "Planned Pitch Velocity: " << plan.pitch_vel << " rad/s\n";
        std::cout << "Planned Pitch Acceleration: " << plan.pitch_acc << " rad/s²\n";
        
        std::cout << "\nFire Command: " << (plan.fire ? "YES" : "NO") << "\n";
        std::cout << "Fired: " << (fired ? "YES" : "NO") << "\n";
        std::cout << "Bullet Speed: " << gs.bullet_speed << " m/s\n";
        if (target.has_value()) {
            std::cout << "\nTarget Z: " << target->ekf_x()[4] << " m\n";
            std::cout << "Target VZ: " << target->ekf_x()[5] << " m/s\n";
            std::cout << "Target W: " << target->ekf_x()[7] << "\n";
        } else {
            std::cout << "\nNo target detected\n";
        }
        std::cout << "===================================\n";
    }
    counter++;
      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);
    if (record) recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();
      auto_aim::Plan plan_snapshot;
      {
        std::lock_guard<std::mutex> lock(latest_plan_mutex);
        plan_snapshot = latest_plan;
      }

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      if (plan_snapshot.control) {
        const auto & aim_xyza = plan_snapshot.aim_xyza;
        auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 0, 255});

        constexpr double ray_length = 10.0;
        auto command_point = tools::ypd2xyz(
          {plan_snapshot.yaw, -plan_snapshot.pitch, ray_length});
        auto command_pixels = solver.world2pixel(
          {{static_cast<float>(command_point.x()), static_cast<float>(command_point.y()),
            static_cast<float>(command_point.z())}});
        if (!command_pixels.empty()) {
          const cv::Point center(command_pixels.front());
          constexpr int marker_size = 10;
          const cv::Scalar marker_color(255, 255, 0);
          cv::line(
            img, center - cv::Point(marker_size, 0), center + cv::Point(marker_size, 0),
            marker_color, 2);
          cv::line(
            img, center - cv::Point(0, marker_size), center + cv::Point(0, marker_size),
            marker_color, 2);
        }
      }
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}