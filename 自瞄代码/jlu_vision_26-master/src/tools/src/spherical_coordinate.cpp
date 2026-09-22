#include "math/spherical_coordinate.hpp"

#include <Eigen/Dense>

Eigen::Vector3d tools::cartesian2Spherical(const Eigen::Vector3d &xyz) {
  auto x{xyz.x()}, y{xyz.y()}, z{xyz.z()};
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

Eigen::Matrix3d tools::cartesian2SphericalJacobian(const Eigen::Vector3d &xyz) {
  auto x{xyz.x()}, y{xyz.y()}, z{xyz.z()};
  auto dyaw_dx = -y / (x * x + y * y);
  auto dyaw_dy = x / (x * x + y * y);
  auto dyaw_dz{0.0};
  auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) *
                               std::pow((x * x + y * y), 1.5));
  auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) *
                               std::pow((x * x + y * y), 1.5));
  auto dpitch_dz =
      1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));
  auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);
  Eigen::Matrix3d J{
      {dyaw_dx, dyaw_dy, dyaw_dz},
      {dpitch_dx, dpitch_dy, dpitch_dz},
      {ddistance_dx, ddistance_dy, ddistance_dz},
  };
  return J;
}
