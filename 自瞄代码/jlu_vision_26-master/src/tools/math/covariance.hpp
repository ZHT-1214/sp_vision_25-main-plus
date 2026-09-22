#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <optional>
#include <type_traits>

namespace tools {
template <int DataSize, int BuffSize, int MinBuffSize = 2>
class SlidingWindowCovarianceCalculator {
public:
  SlidingWindowCovarianceCalculator() = default;

  template <typename... Args>
    requires(sizeof...(Args) == DataSize &&
             (std::is_same_v<Args, double> && ...))
  std::optional<Eigen::Matrix<double, DataSize, DataSize>>
  operator()(Args... args) {
    buffer_.col(count_ % BuffSize) =
        Eigen::Vector<double, DataSize>(static_cast<double>(args)...);
    ++count_;
    auto n = std::min(count_, BuffSize);
    if (n < MinBuffSize)
      return std::nullopt;
    Eigen::Matrix<double, DataSize, Eigen::Dynamic> active =
        buffer_.leftCols(n);
    Eigen::Vector<double, DataSize> mean = active.rowwise().mean();
    Eigen::Matrix<double, DataSize, Eigen::Dynamic> centered =
        (active.colwise() - mean).eval();
    return (centered * centered.transpose()) / (n - 1);
  };

private:
  Eigen::Matrix<double, DataSize, Eigen::Dynamic> buffer_{DataSize, BuffSize};
  int count_{0};
};
} // namespace tools
