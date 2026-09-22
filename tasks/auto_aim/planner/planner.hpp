#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control = false;
  bool fire = false;
  float target_yaw = 0;
  float target_pitch = 0;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
  Eigen::Vector4d aim_xyza = Eigen::Vector4d::Zero();
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);

  // now 必须是该帧目标状态对应的时间戳，用于计算预测提前量。
  // 默认墙钟只适用于实车（相机时间戳与墙钟同源）；离线回放读视频/录制文件时，
  // 视频时间戳与墙钟无关，必须显式传入帧时间戳，否则会算出巨大的 dt 把状态打飞。
  Plan plan(
    std::optional<Target> target, double bullet_speed,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double spin_speed_scale_, min_aim_radius_ratio_, leaving_angle_, min_shoot_angle_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  int choose_armor_id(const Target & target) const;
  bool armor_is_hittable(const Target & target, int armor_id) const;
  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP