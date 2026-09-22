#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
  std::optional<double> nis_threshold_override)
{
  return update(
    z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract, nis_threshold_override);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
  std::optional<double> nis_threshold_override)
{
  // per-call 阈值覆盖，缺省时回落到成员默认值
  double nis_thr = nis_threshold_override.value_or(nis_threshold);

  // 先验状态与协方差（卡方检验需用先验量，故在更新前保存）
  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd P_prior = P;

  // 新息 = 观测量 - 先验状态的观测预测（z - h(x_prior)）
  Eigen::VectorXd innovation = z_subtract(z, h(x_prior));

  // 新息协方差与卡尔曼增益（均基于先验协方差）
  Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  Eigen::MatrixXd K = P_prior * H.transpose() * S.inverse();

  // Stable Computation of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x_prior, K * innovation);

  /// 卡方检验（NIS 用新息 + 先验新息协方差；NEES 用状态修正 + 先验协方差）
  double nis = innovation.transpose() * S.inverse() * innovation;
  double nees = (x - x_prior).transpose() * P_prior.inverse() * (x - x_prior);

  // 卡方检验阈值使用成员变量（nis_threshold / nees_threshold），
  // 默认 0.711，可由使用方按观测维度缩放（见 Target 构造函数）。

  // 每帧先复位失败标志，避免上一次失败后标志永久粘滞
  data["nis_fail"] = 0;
  data["nees_fail"] = 0;
  if (nis > nis_thr) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_thr ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  // 观测维度可能是 1/3/4/8 维，按维度安全写入，缺省补 0
  data["residual_yaw"] = innovation.size() > 0 ? innovation[0] : 0.0;
  data["residual_pitch"] = innovation.size() > 1 ? innovation[1] : 0.0;
  data["residual_distance"] = innovation.size() > 2 ? innovation[2] : 0.0;
  data["residual_angle"] = innovation.size() > 3 ? innovation[3] : 0.0;
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools