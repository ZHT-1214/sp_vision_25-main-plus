#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Solver;

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, Solver & solver, double height_diff = 0);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  void set_stationary(bool stationary, bool lock_center = false);

  bool checkinit();

  // 马氏距离门控参数（由 configs yaml 的 tracker 段加载，见 standard3.yaml）
  double gate_threshold_ = 13.28;           // 门控阈值 = chi2(4 自由度, 99%)
  double gate_yaw_noise_ = 0.004;           // yaw 观测噪声方差 (rad^2)
  double gate_pitch_noise_ = 0.004;         // pitch 观测噪声方差 (rad^2)
  double gate_distance_noise_ratio_ = 0.05; // 距离噪声 std = 比例 × 距离
  double gate_angle_noise_ = 0.09;          // 装甲板 yaw 噪声方差 (rad^2)

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;
  bool stationary_ = false;
  bool lock_center_ = false;
  bool center_locked_ = false;
  int center_samples_ = 0;
  Eigen::Vector3d center_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_anchor_ = Eigen::Vector3d::Zero();

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
  Solver * solver_ = nullptr;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_reprojection(const Armor & armor, int id);  // 图像重投影残差观测

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
  double gate_mahalanobis(const Armor & armor, int & matched_id) const;
  Eigen::VectorXd h_reproject(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_reproject_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP