#include "target.hpp"

#include <numeric>

#include "solver.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, Solver & solver, double height_diff)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0),
  solver_(&solver)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, height_diff}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
  // 8 维像素重投影观测：NIS 阈值随自由度线性缩放（0.711 / 4 * 8），需实车标定
  ekf_.nis_threshold = 1.422;
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4), solver_(nullptr)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
  // 8 维像素重投影观测：NIS 阈值随自由度线性缩放（0.711 / 4 * 8），需实车标定
  ekf_.nis_threshold = 1.422;
}

void Target::set_stationary(bool stationary, bool lock_center)
{
  stationary_ = stationary;
  lock_center_ = lock_center;
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  // 正常跟踪时 dt 应为帧间隔（毫秒级）。dt 过大说明传入的时间戳与 t_ 不同源
  // （例如离线回放混用了墙钟），此时 Q 会随 dt^4 爆炸并直接打飞状态，故必须报警。
  if (std::abs(dt) > 1.0) tools::logger()->warn("[Target] Large predict dt: {:.3f}s", dt);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = stationary_ ? 5 : 100;  // 静止小陀螺降低中心位置漂移
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
  if (stationary_) {
    ekf_.x[1] = 0;
    ekf_.x[3] = 0;
    ekf_.x[5] = 0;
  }
}

double Target::gate_mahalanobis(const Armor & armor, int & matched_id) const
{
  // 观测 ypd（yaw/pitch/distance）+ 装甲板 yaw（世界系）
  const Eigen::Vector3d & obs_ypd = armor.ypd_in_world;
  const double obs_yaw = armor.ypr_in_world[0];

  double min_d2 = 1e18;
  matched_id = -1;

  for (int id = 0; id < armor_num_; ++id) {
    // 预测装甲板 ypd + yaw
    Eigen::Vector3d pred_xyz = h_armor_xyz(ekf_.x, id);
    Eigen::Vector3d pred_ypd = tools::xyz2ypd(pred_xyz);
    double pred_yaw = tools::limit_rad(ekf_.x[6] + id * 2 * CV_PI / armor_num_);

    // 新息（角度分量归一化到 [-pi, pi]）
    Eigen::Vector4d diff;
    diff[0] = tools::limit_rad(obs_ypd[0] - pred_ypd[0]);  // yaw
    diff[1] = obs_ypd[1] - pred_ypd[1];                     // pitch
    diff[2] = obs_ypd[2] - pred_ypd[2];                     // distance
    diff[3] = tools::limit_rad(obs_yaw - pred_yaw);         // 装甲板 yaw

    // 新息协方差 S = H P H^T + R（H 为 ypda 观测雅可比）
    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);  // 4x11
    Eigen::MatrixXd S = H * ekf_.P * H.transpose();
    double dist = std::max(obs_ypd[2], 0.5);
    S(0, 0) += gate_yaw_noise_;                                     // yaw 观测噪声方差
    S(1, 1) += gate_pitch_noise_;                                   // pitch
    S(2, 2) += gate_distance_noise_ratio_ * dist * gate_distance_noise_ratio_ * dist;  // distance
    S(3, 3) += gate_angle_noise_;                                   // 装甲板 yaw

    double d2 = diff.transpose() * S.inverse() * diff;
    if (d2 < min_d2) {
      min_d2 = d2;
      matched_id = id;
    }
  }

  return min_d2;
}

void Target::update(const Armor & armor)
{
  // 马氏距离门控关联：选马氏距离最小的装甲板 id，超过阈值则拒绝该观测
  int id = -1;
  double gate = gate_mahalanobis(armor, id);
  if (id < 0 || gate > gate_threshold_) {
    tools::logger()->debug("[Target] gated out, mahalanobis={:.2f}", gate);
    return;
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  update_ypda(armor, id);
  if (stationary_ && lock_center_ && !center_locked_) {
    center_sum_.x() += ekf_.x[0];
    center_sum_.y() += ekf_.x[2];
    center_sum_.z() += ekf_.x[4];
    center_samples_++;
    if (center_samples_ >= 15) {
      center_anchor_ = center_sum_ / center_samples_;
      center_locked_ = true;
    }
  }
  if (stationary_ && lock_center_ && center_locked_) {
    ekf_.x[0] = center_anchor_.x();
    ekf_.x[2] = center_anchor_.y();
    ekf_.x[4] = center_anchor_.z();
  }
}

void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
}

void Target::update_reprojection(const Armor & armor, int id)
{
  if (solver_ == nullptr) return;

  // 观测量：检测到的四个角点像素坐标（顺序与 solver 的 object_points 一致）
  Eigen::VectorXd z(8);
  for (int i = 0; i < 4; ++i) {
    z(2 * i) = armor.points[i].x;
    z(2 * i + 1) = armor.points[i].y;
  }

  // 观测雅可比（数值微分）
  Eigen::MatrixXd H = h_reproject_jacobian(ekf_.x, id);

  // 像素级观测噪声：对角阵，σ = 2 px
  constexpr double SIGMA_PX = 2.0;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(8, 8) * (SIGMA_PX * SIGMA_PX);

  // 非线性观测函数 x -> z（像素坐标，直接相减，无需角度包裹）
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd { return h_reproject(x, id); };

  ekf_.update(z, H, R, h);
}

Eigen::VectorXd Target::h_reproject(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto center = h_armor_xyz(x, id);
  auto pts = solver_->reproject_armor(center, angle, armor_type, name);

  Eigen::VectorXd z(8);
  for (int i = 0; i < 4; ++i) {
    z(2 * i) = pts[i].x;
    z(2 * i + 1) = pts[i].y;
  }
  return z;
}

Eigen::MatrixXd Target::h_reproject_jacobian(const Eigen::VectorXd & x, int id) const
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(8, x.size());

  // 只有位置、yaw、半径、高度差直接影响投影；速度与角速度列恒为 0
  constexpr int OBSERVED_IDX[] = {0, 2, 4, 6, 8, 9, 10};
  for (int idx : OBSERVED_IDX) {
    // 位置扰动 1e-3 m，yaw/半径/高度差扰动 1e-4
    double eps = (idx == 0 || idx == 2 || idx == 4) ? 1e-3 : 1e-4;

    Eigen::VectorXd x_plus = x;
    Eigen::VectorXd x_minus = x;
    x_plus[idx] += eps;
    x_minus[idx] -= eps;

    H.col(idx) = (h_reproject(x_plus, id) - h_reproject(x_minus, id)) / (2 * eps);
  }
  return H;
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
