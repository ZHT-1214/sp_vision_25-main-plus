#pragma once

#include <Eigen/Dense>

namespace tools {
namespace ypd {
enum {
  YAW,
  PITCH,
  DISTANCE,
};
}
Eigen::Vector3d cartesian2Spherical(const Eigen::Vector3d &xyz);
Eigen::Matrix3d cartesian2SphericalJacobian(const Eigen::Vector3d &xyz);

} // namespace tools
