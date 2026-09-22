// Copyright (c) 2026 shuodedaoli. All Rights Reserved.
#include "target.hpp"
#include "configs.hpp"
#include "factors.hpp"
#include "math/angle_tools.hpp"
#include "math/sigmoid_functions.hpp"
#include "math/spherical_coordinate.hpp"
#include "types.hpp"
#include "types/Armor.hpp"
#include "types/ArmorType.hpp"

#include "quill/LogMacros.h"
#include "rfl/enums.hpp"
#include <cstdlib>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iterator>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

using namespace gtsam::symbol_shorthand;

std::vector<auto_aim::ArmorMatchResult> auto_aim::Target::matchArmor(
    const std::vector<ArmorPositionYaw> &armors, const ArmorPositionYaw &obs,
    double max_match_distance, double max_match_yaw_diff) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto &armor : armors) {
    auto distance = (armor.position - obs.position).norm();
    auto yaw_diff = std::abs(obs.yaw.localCoordinates(armor.yaw).x());
    if (distance <= max_match_distance && yaw_diff <= max_match_yaw_diff)
      results.emplace_back(static_cast<ArmorIndex>(i), distance, yaw_diff);
    ++i;
  }
  // 暂时先选择最面对的
  std::ranges::sort(results, std::ranges::less{}, &ArmorMatchResult::yaw_diff);
  return results;
}

std::vector<auto_aim::ArmorMatchResult> auto_aim::Target::matchArmor(
    const std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>
        &armors_covs,
    const ArmorPositionYaw &obs) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto &[armor, cov] : armors_covs) {
    Eigen::Vector3d dypd = tools::cartesian2Spherical(obs.position) -
                           tools::cartesian2Spherical(armor.position);
    auto dr = armor.yaw.localCoordinates(obs.yaw).x();
    Eigen::Vector4d error{Eigen::Vector4d::Zero()};
    error.head<3>() = dypd;
    error(3) = dr;
    double mahalanobis_distance = error.transpose() * cov.ldlt().solve(error);
    results.emplace_back(static_cast<ArmorIndex>(i), mahalanobis_distance, dr);
    ++i;
  }
  // 马氏距离贪心
  std::ranges::sort(results, std::ranges::less{}, &ArmorMatchResult::distance);
  return results;
}

auto_aim::RobotTarget::RobotTarget(quill::Logger *logger,
                                   const RobotConfig &config,
                                   types::ArmorType type,
                                   const cv::Mat &camera_matrix,
                                   const cv::Mat &distortion_coefficients)
    : logger_(logger), config_(config), camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients) {
  target_state_.type = type;
  target_state_.radius_a = config_.default_radius;
  target_state_.radius_b = config_.default_radius;
  target_state_.dz = config_.default_dz;
  track_state_.state = TrackState::State::LOST;
  track_state_.stamp_last_update = std::chrono::system_clock::from_time_t(0);
  track_state_.stamp_last_tracking = std::chrono::system_clock::from_time_t(0);
  track_state_.k = 0;
  this->resetCovariances();
}

std::vector<auto_aim::ArmorPositionYaw>
auto_aim::RobotTarget::getArmorsFromTargetState(const TargetState &state,
                                                double radius_a,
                                                double radius_b, double dz) {
  std::vector<ArmorPositionYaw> armors;
  for (auto i : std::array{
           ArmorIndex::_0,
           ArmorIndex::_1,
           ArmorIndex::_2,
           ArmorIndex::_3,
       }) {
    auto [_r, _dz] = (i == ArmorIndex::_0 || i == ArmorIndex::_2)
                         ? std::make_pair(radius_a, 0.0)
                         : std::make_pair(radius_b, dz);
    // NOTE: 好像只在这里用到了positionyaw对center状态的构造函数
    armors.emplace_back(state.center_position, state.center_yaw, _r, _dz, 4, i);
  }
  return armors;
}

std::vector<auto_aim::ArmorPositionYaw>
auto_aim::RobotTarget::getArmorsFromTargetState(
    const RobotTargetState &state) const {
  return RobotTarget::getArmorsFromTargetState(state, state.radius_a,
                                               state.radius_b, state.dz);
}

std::vector<
    std::pair<auto_aim::ArmorPositionRollPitchYawPoints, auto_aim::ArmorIndex>>
auto_aim::RobotTarget::matchArmors(
    const RobotTargetState &state, double dt,
    const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_camera,
    const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_odom) const {
  Eigen::Matrix3d cov_X_pred = X_cov_ + V_cov_ * (dt * dt);
  cov_X_pred.diagonal() += Eigen::Vector3d{
      config_.translation_factor_noise.x * dt * dt,
      config_.translation_factor_noise.y * dt * dt,
      config_.translation_factor_noise.z * dt * dt,
  };
  double cov_R_pred = R_cov_ + W_cov_ * dt * dt;
  cov_R_pred += config_.yaw_factor_noise * dt * dt;
  Eigen::Matrix4d center_cov = Eigen::Matrix4d::Zero();
  center_cov.topLeftCorner<3, 3>() = cov_X_pred;
  center_cov(3, 3) = cov_R_pred;
  auto armors = RobotTarget::getArmorsFromTargetState(state);
  // HACK: 因为S中的R对每个观测都不同，即需要区分不同观测的四个协方差
  // 先拿个vec装着了，后续重构下接口最好
  std::vector<std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>>
      armors_covs_vec;
  for (const auto &obs : obs_armors_odom) {
    std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>> armors_covs;
    for (const auto &armor : armors) {
      Eigen::Vector3d offset = armor.position - state.center_position;
      Eigen::Matrix4d J = Eigen::Matrix4d::Identity();
      J(0, 3) = -offset.y();
      J(1, 3) = offset.x();
      Eigen::Matrix4d P = J * center_cov * J.transpose();
      // 将xyz协方差转换成ypd协方差
      Eigen::Matrix3d P_xyz = P.topLeftCorner<3, 3>();
      Eigen::Matrix3d J_sph =
          tools::cartesian2SphericalJacobian(armor.position);
      P.topLeftCorner<3, 3>() = J_sph * P_xyz * J_sph.transpose(); // P_ypd
      auto obs_center_yaw = std::atan2(obs.position.y(), obs.position.x());
      auto obs_incline_angle =
          tools::limitRadian(obs.yaw.theta() - obs_center_yaw);
      // XXX: 没有人类了
      Eigen::VectorXd R_dig{
          {config_.armor_match_conf.ypd_conf.yaw_pitch_noise,
           config_.armor_match_conf.ypd_conf.yaw_pitch_noise,
           log(config_.armor_match_conf.ypd_conf.distance_noise_log_scale *
                   std::abs(obs_incline_angle) +
               1) +
               config_.armor_match_conf.ypd_conf.basic_distance_noise,
           log(std::abs(obs.position.norm()) + 1) /
                   config_.armor_match_conf.ypd_conf.armor_yaw_log_divisor +
               config_.armor_match_conf.ypd_conf.basic_armor_yaw_noise},
      };
      Eigen::Matrix4d R = R_dig.asDiagonal();
      Eigen::Matrix4d S = P + R;
      armors_covs.emplace_back(armor, S);
    }
    armors_covs_vec.emplace_back(armors_covs);
  }
  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
      matched_armors;
  std::array<std::vector<std::pair<double, ArmorPositionRollPitchYawPoints>>, 4>
      index_vector_array;
  for (std::size_t i = 0;
       i < obs_armors_camera.size() && i < obs_armors_odom.size(); ++i) {
    const auto &obs = obs_armors_odom.at(i);
    auto result = Target::matchArmor(armors_covs_vec.at(i), obs);
    if (result.empty())
      continue;
    // NOTE: 在此处从马氏距离更新NIS失败队列，相信只要是观测就都是有效的
    this->updateNisFailureDeque(result.front().distance);
    if (result.front().distance <
        config_.armor_match_conf.max_match_mahalanobis_distance) {
      index_vector_array.at(static_cast<int>(result.front().index))
          .emplace_back(result.front().distance, obs_armors_camera.at(i));
    }
  }
  int idx = 0;
  for (auto &cost_armor_vec : index_vector_array) {
    std::ranges::sort(
        cost_armor_vec, std::less<>{},
        &std::pair<double, ArmorPositionRollPitchYawPoints>::first);
    if (!cost_armor_vec.empty())
      matched_armors.emplace_back(std::pair{cost_armor_vec.front().second,
                                            static_cast<ArmorIndex>(idx)});
    idx++;
  }
  return matched_armors;
}

void auto_aim::RobotTarget::updateNisFailureDeque(double distance) const {
  this->nis_failure_deque_.push_back(distance >
                                     config_.nis_conf.nis_failure_thres);
  if (nis_failure_deque_.size() > config_.nis_conf.nis_failure_window_size)
    nis_failure_deque_.pop_front();
}

std::optional<bool> auto_aim::RobotTarget::nisFailured() const {
  if (nis_failure_deque_.size() < config_.nis_conf.nis_failure_window_size)
    return std::nullopt;
  return (static_cast<double>(std::ranges::count(nis_failure_deque_, true)) /
          config_.nis_conf.nis_failure_window_size) >
         (config_.nis_conf.reset_failure_percentage_thres / 100.0);
}

void auto_aim::OutpostTarget::updateNisFailureDeque(double distance) const {
  this->nis_failure_deque_.push_back(distance >
                                     config_.nis_conf.nis_failure_thres);
  if (nis_failure_deque_.size() > config_.nis_conf.nis_failure_window_size)
    nis_failure_deque_.pop_front();
}

std::optional<bool> auto_aim::OutpostTarget::nisFailured() const {
  if (nis_failure_deque_.size() < config_.nis_conf.nis_failure_window_size)
    return std::nullopt;
  return (static_cast<double>(std::ranges::count(nis_failure_deque_, true)) /
          config_.nis_conf.nis_failure_window_size) >
         (config_.nis_conf.reset_failure_percentage_thres / 100.0);
}

auto_aim::RobotTargetState auto_aim::RobotTarget::getTargetStateFromArmor(
    const ArmorPositionYaw &armor) const {
  auto armor_x = armor.position.x();
  auto armor_y = armor.position.y();
  auto center_x =
      armor_x + config_.default_radius * std::cos(armor.yaw.theta());
  auto center_y =
      armor_y + config_.default_radius * std::sin(armor.yaw.theta());
  auto center_z = armor.position.z();
  RobotTargetState state;
  state.type = target_state_.type;
  state.center_position = Eigen::Vector3d{center_x, center_y, center_z};
  state.center_yaw = armor.yaw.theta();
  state.radius_a = config_.default_radius;
  state.radius_b = config_.default_radius;
  state.dz = config_.default_dz;
  return state;
}

std::pair<auto_aim::TargetState, auto_aim::TrackState>
auto_aim::RobotTarget::getTargetTrackState() const {
  std::scoped_lock lk{state_mtx_};
  return {
      target_state_.getStateWithArmorsFunc(
          [ra = target_state_.radius_a, rb = target_state_.radius_b,
           dz = target_state_.dz](const TargetState &state) {
            return getArmorsFromTargetState(state, ra, rb, dz);
          }),
      track_state_,
  };
}

auto_aim::TrackState::State
auto_aim::RobotTarget::track(const std::vector<types::Armor> &armors,
                             const std::chrono::system_clock::time_point &stamp,
                             const Eigen::Isometry3d &T_camera_to_odom) {
  std::vector<ArmorPositionRollPitchYawPoints> selected_armors;
  std::copy_if(armors.begin(), armors.end(),
               std::back_inserter(selected_armors),
               [this](const types::Armor &armor) -> bool {
                 return armor.type == target_state_.type;
               }); // 隐式调用构造函数
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  stamp - track_state_.stamp_last_update)
                  .count();
  auto [estimated_target_state, updated_track_state] =
      update(selected_armors, dt, T_camera_to_odom);
  if (updated_track_state == TrackState::State::TRACKING) {
    if (track_state_.state != TrackState::State::TRACKING) {
      LOG_INFO(logger_, "[Target {}]: {} -> TRACKING. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type),
               rfl::enum_to_string(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_tracking = stamp;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::TEMPLOST) {
    if (track_state_.state != TrackState::State::TEMPLOST) {
      LOG_INFO(logger_, "[Target {}]: TRACKING -> TEMPLOST. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::LOST) {
    if (track_state_.state != TrackState::State::LOST) {
      LOG_INFO(logger_, "[Target {}]: {} -> LOST. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type),
               rfl::enum_to_string(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    track_state_.state = TrackState::State::LOST;
    track_state_.k = 0;
    isam2_ = gtsam::ISAM2{};
    this->resetCovariances();
    this->nis_failure_deque_.clear();
  }
  return track_state_.state;
}

double auto_aim::RobotTarget::get(const std::string &key) const {
  std::scoped_lock lk{state_mtx_};
  if (key == "ra")
    return target_state_.radius_a;
  if (key == "rb")
    return target_state_.radius_b;
  if (key == "dz")
    return target_state_.dz;
  return 0;
}

inline gtsam::Key getArmorPoseKeyFromIndex(auto_aim::ArmorIndex index,
                                           std::uint64_t k) {
  if (index == auto_aim::ArmorIndex::_0)
    return H(k);
  if (index == auto_aim::ArmorIndex::_1)
    return J(k);
  if (index == auto_aim::ArmorIndex::_2)
    return K(k);
  if (index == auto_aim::ArmorIndex::_3)
    return L(k);
  // 非常好索引，爱来自Vim
  return {};
}

void auto_aim::RobotTarget::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const TargetState &target_state, std::uint64_t k, double dt) const {
  values.insert(X(k), target_state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(target_state.center_yaw));
  values.insert(V(k), target_state.center_velocity);
  values.insert(W(k), target_state.center_vyaw);
  if (k == 0) {
    graph.addPrior(X(0), target_state.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.translation_prior_noise.x,
                       config_.translation_prior_noise.y,
                       config_.translation_prior_noise.z,
                   }));
    graph.addPrior(
        R(0), gtsam::Rot2::fromAngle(target_state.center_yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_prior_noise));
    graph.addPrior(V(0), target_state.center_velocity,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.velocity_prior_noise.x,
                       config_.velocity_prior_noise.y,
                       config_.velocity_prior_noise.z,
                   }));
    graph.addPrior(
        W(0), target_state.center_vyaw,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_prior_noise));
  } else {
    graph.add(TranslationFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.translation_factor_noise.x,
            config_.translation_factor_noise.y,
            config_.translation_factor_noise.z,
        }),
        X(k - 1),
        V(k - 1),
        X(k),
        dt,
    });
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(k - 1),
        W(k - 1),
        R(k),
        dt,
    });
    graph.add(VelocityFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.velocity_factor_noise.x,
            config_.velocity_factor_noise.y,
            config_.velocity_factor_noise.z,
        }),
        V(k - 1),
        V(k),
    });
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_factor_noise),
        W(k - 1),
        W(k),
    });
  }
  LOG_TRACE_L1(logger_,
               "[Target {}]: add motion values factors. k = {}, dt = {}.",
               rfl::enum_to_string(target_state_.type), k, dt);
}

void auto_aim::RobotTarget::addArmorReprojValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    gtsam::Key armor_pose_key, const ArmorPositionRollPitchYawPoints &armor,
    std::uint64_t k) const {
  values.insert(armor_pose_key,
                gtsam::Pose3{gtsam::Rot3{armor.getRotation()}, armor.position});
  for (auto position : std::array{
           types::ArmorPointPosition::LeftBottom,
           types::ArmorPointPosition::LeftTop,
           types::ArmorPointPosition::RightTop,
           types::ArmorPointPosition::RightBottom,
       }) {
    graph.add(ArmorReprojFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2{
            config_.armor_observation_noise.pixel_error.x,
            config_.armor_observation_noise.pixel_error.y,
        }),
        armor_pose_key,
        camera_matrix_,
        distortion_coefficients_,
        target_state_.type,
        position,
        armor.points.at(static_cast<int>(position)),
    });
  }
}

void auto_aim::RobotTarget::addArmorValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
        &armors_indexs,
    const Eigen::Isometry3d &T, std::uint64_t k) const {
  auto default_radius = tools::logisticInverse(
      config_.default_radius, config_.radius_min, config_.radius_max);
  if (k == 0) {
    values.insert(A(0), default_radius);
    values.insert(B(0), default_radius);
    values.insert(Z(0), config_.default_dz);
    graph.addPrior(
        A(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        B(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        Z(0), config_.default_dz,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.dz_prior_noise));
  }
  for (const auto &[armor, index] : armors_indexs) {
    auto armor_pose_key = getArmorPoseKeyFromIndex(index, k);
    addArmorReprojValuesFactors(values, graph, armor_pose_key, armor, k);
    if (index == ArmorIndex::_0 || index == ArmorIndex::_2) {
      graph.add(ArmorRadiusCenterZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.armor_observation_noise.tangential_error_m,
              config_.armor_observation_noise.radial_error_m,
              config_.armor_observation_noise.height_error_m,
              config_.armor_observation_noise.yaw_error_rad,
          }),
          armor_pose_key,
          A(0),
          R(k),
          X(k),
          T,
          index,
          config_.radius_min,
          config_.radius_max,
      });
    } else {
      graph.add(ArmorRadiusDZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.armor_observation_noise.tangential_error_m,
              config_.armor_observation_noise.radial_error_m,
              config_.armor_observation_noise.height_error_m,
              config_.armor_observation_noise.yaw_error_rad,
          }),
          armor_pose_key,
          B(0),
          Z(0),
          R(k),
          X(k),
          T,
          index,
          config_.radius_min,
          config_.radius_max,
      });
    }
  }
  LOG_TRACE_L1(logger_, "[Target {}]: add {} armor factors. k = {}.",
               rfl::enum_to_string(target_state_.type), armors_indexs.size(),
               k);
}

std::pair<auto_aim::RobotTargetState, auto_aim::TrackState::State>
auto_aim::RobotTarget::update(
    const std::vector<ArmorPositionRollPitchYawPoints> &armors, double dt,
    const Eigen::Isometry3d &T) const {
  auto dt_tracking_to_update =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          track_state_.stamp_last_update - track_state_.stamp_last_tracking)
          .count();
  if (track_state_.state != TrackState::State::LOST &&
      dt + dt_tracking_to_update > config_.lost_threshold_sec) {
    LOG_INFO(logger_, "[Target {}]: Time out! dt{}.",
             rfl::enum_to_string(target_state_.type),
             dt + dt_tracking_to_update);
    return {{}, TrackState::State::LOST};
  }
  if (auto nis_fail_opt = nisFailured();
      nis_fail_opt.has_value() && nis_fail_opt.value()) {
    LOG_INFO(logger_, "[Target {}]: Nis failure!",
             rfl::enum_to_string(target_state_.type));
    return {{}, TrackState::State::LOST};
  }
  auto obs_armors_odom = armors;
  for (auto &armor : obs_armors_odom) {
    Eigen::Isometry3d armor_pose_camera{Eigen::Isometry3d::Identity()};
    armor_pose_camera.pretranslate(armor.position);
    armor_pose_camera.rotate(armor.getRotation());
    auto armor_pose_odom = T * armor_pose_camera;
    auto rpy = tools::rotationMatrixToRPY(armor_pose_odom.rotation());
    armor.position = armor_pose_odom.translation();
    armor.roll = rpy.x();
    armor.pitch = rpy.y();
    armor.yaw = gtsam::Rot2::fromAngle(rpy.z());
  }

  auto target_state = target_state_.predict(dt);
  if (track_state_.state == TrackState::State::LOST) {
    if (obs_armors_odom.empty())
      return {{}, TrackState::State::LOST};
    else
      target_state = getTargetStateFromArmor(obs_armors_odom.front());
  }

  if (std::abs(target_state.center_vyaw) > config_.huge_vyaw_reset_thres) {
    LOG_INFO(logger_, "[Target {}]: Abnormal vyaw {}!",
             rfl::enum_to_string(target_state.type), target_state.center_vyaw);
    return {{}, TrackState::State::LOST};
  }

  // XXX: 这里似乎没有考虑到冷启动过程中的临时丢失
  if (track_state_.k < config_.first_update_batch_size &&
      !obs_armors_odom.empty()) {
    target_state = getTargetStateFromArmor(obs_armors_odom.front());
  }

  auto matched_armors = matchArmors(target_state, dt, armors, obs_armors_odom);
  if (matched_armors.size() < obs_armors_odom.size()) {
    LOG_DEBUG(logger_, "[Target {}]: Miss match {} armors! k = {}.",
              rfl::enum_to_string(target_state.type),
              obs_armors_odom.size() - matched_armors.size(), track_state_.k);
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  if (track_state_.k <= config_.first_update_batch_size) {
    if (track_state_.k == 0) {
      this->initial_values_ = gtsam::Values{};
      this->initial_graph_ = gtsam::NonlinearFactorGraph{};
    }
    values = this->initial_values_;
    graph = this->initial_graph_;
  }
  addMotionValuesFactors(values, graph, target_state, track_state_.k, dt);
  addArmorValuesFactors(values, graph, matched_armors, T, track_state_.k);

  // NOTE: 冷启动时先不更新到isam2里(避免在缺少约束前几帧优化爆掉)
  if (track_state_.k < config_.first_update_batch_size) {
    this->initial_values_ = values;
    this->initial_graph_ = graph;
    return {target_state, matched_armors.empty() ? TrackState::State::TEMPLOST
                                                 : TrackState::State::TRACKING};
  }

  try {
    this->isam2_.update(graph, values);
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_velocity =
        isam2_.calculateEstimate<gtsam::Vector3>(V(track_state_.k));
    target_state.center_yaw =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vyaw =
        isam2_.calculateEstimate<double>(W(track_state_.k));
    target_state.radius_a =
        tools::logisticFunction(isam2_.calculateEstimate<double>(A(0)),
                                config_.radius_min, config_.radius_max);
    target_state.radius_b =
        tools::logisticFunction(isam2_.calculateEstimate<double>(B(0)),
                                config_.radius_min, config_.radius_max);
    target_state.dz = isam2_.calculateEstimate<double>(Z(0));
    // NOTE: isam2计算联合协方差的jointMarginalCovariance方法还在feature分支上
    // 先用协方差凑合一下吧，等一手gtsam更新
    this->X_cov_ = isam2_.marginalCovariance(X(track_state_.k));
    this->V_cov_ = isam2_.marginalCovariance(V(track_state_.k));
    this->R_cov_ = isam2_.marginalCovariance(R(track_state_.k))(0, 0);
    this->W_cov_ = isam2_.marginalCovariance(W(track_state_.k))(0, 0);
    return {target_state, matched_armors.empty() ? TrackState::State::TEMPLOST
                                                 : TrackState::State::TRACKING};
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "[Target {}]: {}\ncurrent k: {}.",
              rfl::enum_to_string(target_state.type), e.what(), track_state_.k);
    return {{}, TrackState::State::LOST};
  }
}

void auto_aim::RobotTarget::resetCovariances() {
  X_cov_ = Eigen::Matrix3d::Zero();
  X_cov_(0, 0) =
      config_.translation_prior_noise.x * config_.translation_prior_noise.x;
  X_cov_(1, 1) =
      config_.translation_prior_noise.y * config_.translation_prior_noise.y;
  X_cov_(2, 2) =
      config_.translation_prior_noise.z * config_.translation_prior_noise.z;
  V_cov_ = Eigen::Matrix3d::Zero();
  V_cov_(0, 0) =
      config_.velocity_prior_noise.x * config_.velocity_prior_noise.x;
  V_cov_(1, 1) =
      config_.velocity_prior_noise.y * config_.velocity_prior_noise.y;
  V_cov_(2, 2) =
      config_.velocity_prior_noise.z * config_.velocity_prior_noise.z;
  R_cov_ = config_.yaw_prior_noise * config_.yaw_prior_noise;
  W_cov_ = config_.vyaw_prior_noise * config_.vyaw_prior_noise;
}

void auto_aim::OutpostTarget::resetCovariances() {
  X_cov_ = Eigen::Matrix3d::Zero();
  X_cov_(0, 0) =
      config_.translation_prior_noise.x * config_.translation_prior_noise.x;
  X_cov_(1, 1) =
      config_.translation_prior_noise.y * config_.translation_prior_noise.y;
  X_cov_(2, 2) =
      config_.translation_prior_noise.z * config_.translation_prior_noise.z;
  V_cov_ = Eigen::Matrix3d::Zero();
  V_cov_(0, 0) =
      config_.velocity_prior_noise.x * config_.velocity_prior_noise.x;
  V_cov_(1, 1) =
      config_.velocity_prior_noise.y * config_.velocity_prior_noise.y;
  V_cov_(2, 2) =
      config_.velocity_prior_noise.z * config_.velocity_prior_noise.z;
  R_cov_ = config_.yaw_prior_noise * config_.yaw_prior_noise;
  W_cov_ = config_.vyaw_prior_noise * config_.vyaw_prior_noise;
}

auto_aim::OutpostTarget::OutpostTarget(quill::Logger *logger,
                                       const OutpostConfig &config,
                                       const cv::Mat &camera_matrix,
                                       const cv::Mat &distortion_coefficients)
    : logger_(logger), config_(config), camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients) {
  target_state_.type = types::ArmorType::Outpost;
  target_state_.radius = config_.default_radius;
  target_state_.dz_0 = 0;
  target_state_.dz_1 = 0;
  track_state_.state = TrackState::State::LOST;
  track_state_.stamp_last_update = std::chrono::system_clock::from_time_t(0);
  track_state_.stamp_last_tracking = std::chrono::system_clock::from_time_t(0);
  track_state_.k = 0;
  this->resetCovariances();
}

auto_aim::TrackState::State auto_aim::OutpostTarget::track(
    const std::vector<types::Armor> &armors,
    const std::chrono::system_clock::time_point &stamp,
    const Eigen::Isometry3d &T_camera_to_odom) {
  std::vector<ArmorPositionRollPitchYawPoints> selected_armors;
  std::copy_if(armors.begin(), armors.end(),
               std::back_inserter(selected_armors),
               [this](const types::Armor &armor) -> bool {
                 return armor.type == target_state_.type;
               }); // 隐式调用构造函数
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  stamp - track_state_.stamp_last_update)
                  .count();
  auto [estimated_target_state, updated_track_state] =
      update(selected_armors, dt, T_camera_to_odom);
  if (updated_track_state == TrackState::State::TRACKING) {
    if (track_state_.state != TrackState::State::TRACKING) {
      LOG_INFO(logger_, "[Target {}]: {} -> TRACKING. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type),
               rfl::enum_to_string(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_tracking = stamp;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::TEMPLOST) {
    if (track_state_.state != TrackState::State::TEMPLOST) {
      LOG_INFO(logger_, "[Target {}]: TRACKING -> TEMPLOST. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::LOST) {
    if (track_state_.state != TrackState::State::LOST) {
      LOG_INFO(logger_, "[Target {}]: {} -> LOST. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type),
               rfl::enum_to_string(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    track_state_.state = TrackState::State::LOST;
    track_state_.k = 0;
    this->resetCovariances();
    this->nis_failure_deque_.clear();
    isam2_ = gtsam::ISAM2{};
  }
  return track_state_.state;
}

std::pair<auto_aim::TargetState, auto_aim::TrackState>
auto_aim::OutpostTarget::getTargetTrackState() const {
  std::scoped_lock lk{state_mtx_};
  return {
      target_state_.getStateWithArmorsFunc(
          [r = target_state_.radius, dz_0 = target_state_.dz_0,
           dz_1 = target_state_.dz_1,
           dz_2 = target_state_.dz_2](const TargetState &state) {
            return getArmorsFromTargetState(state, r, dz_0, dz_1, dz_2);
          }),
      track_state_,
  };
}

double auto_aim::OutpostTarget::get(const std::string &key) const {
  std::scoped_lock lk{state_mtx_};
  if (key == "r")
    return target_state_.radius;
  if (key == "dz0")
    return target_state_.dz_0;
  if (key == "dz1")
    return target_state_.dz_1;
  if (key == "dz2")
    return target_state_.dz_2;
  return 0;
}

std::vector<auto_aim::ArmorPositionYaw>
auto_aim::OutpostTarget::getArmorsFromTargetState(const TargetState &state,
                                                  double radius, double dz_0,
                                                  double dz_1, double dz_2) {
  std::vector<ArmorPositionYaw> armors;
  for (auto i : std::array{
           ArmorIndex::_0,
           ArmorIndex::_1,
           ArmorIndex::_2,
       }) {
    auto dz =
        (i == ArmorIndex::_0) ? dz_0 : ((i == ArmorIndex::_1) ? dz_1 : dz_2);
    armors.emplace_back(state.center_position, state.center_yaw, radius, dz, 3,
                        i);
  }
  return armors;
}

std::vector<auto_aim::ArmorPositionYaw>
auto_aim::OutpostTarget::getArmorsFromTargetState(
    const OutpostTargetState &state) const {
  return getArmorsFromTargetState(state, state.radius, state.dz_0, state.dz_1,
                                  state.dz_2);
}

std::vector<
    std::pair<auto_aim::ArmorPositionRollPitchYawPoints, auto_aim::ArmorIndex>>
auto_aim::OutpostTarget::matchArmors(
    const OutpostTargetState &state, double dt,
    const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_camera,
    const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_odom) const {
  Eigen::Matrix3d cov_X_pred = X_cov_ + V_cov_ * (dt * dt);
  cov_X_pred.diagonal() += Eigen::Vector3d{
      config_.translation_factor_noise.x * dt * dt,
      config_.translation_factor_noise.y * dt * dt,
      config_.translation_factor_noise.z * dt * dt,
  };
  double cov_R_pred = R_cov_ + W_cov_ * dt * dt;
  cov_R_pred += config_.yaw_factor_noise * dt * dt;
  Eigen::Matrix4d center_cov = Eigen::Matrix4d::Zero();
  center_cov.topLeftCorner<3, 3>() = cov_X_pred;
  center_cov(3, 3) = cov_R_pred;
  auto armors = OutpostTarget::getArmorsFromTargetState(state);
  // HACK: 因为S中的R对每个观测都不同，即需要区分不同观测的四个协方差
  // 先拿个vec装着了，后续重构下接口最好
  std::vector<std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>>
      armors_covs_vec;
  for (const auto &obs : obs_armors_odom) {
    std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>> armors_covs;
    for (const auto &armor : armors) {
      Eigen::Vector3d offset = armor.position - state.center_position;
      Eigen::Matrix4d J = Eigen::Matrix4d::Identity();
      J(0, 3) = -offset.y();
      J(1, 3) = offset.x();
      Eigen::Matrix4d P = J * center_cov * J.transpose();
      // 将xyz协方差转换成ypd协方差
      Eigen::Matrix3d P_xyz = P.topLeftCorner<3, 3>();
      Eigen::Matrix3d J_sph =
          tools::cartesian2SphericalJacobian(armor.position);
      P.topLeftCorner<3, 3>() = J_sph * P_xyz * J_sph.transpose(); // P_ypd
      auto obs_center_yaw = std::atan2(obs.position.y(), obs.position.x());
      auto obs_incline_angle =
          tools::limitRadian(obs.yaw.theta() - obs_center_yaw);
      // XXX: 没有人类了
      Eigen::VectorXd R_dig{
          {config_.armor_match_conf.ypd_conf.yaw_pitch_noise,
           config_.armor_match_conf.ypd_conf.yaw_pitch_noise,
           log(config_.armor_match_conf.ypd_conf.distance_noise_log_scale *
                   std::abs(obs_incline_angle) +
               1) +
               config_.armor_match_conf.ypd_conf.basic_distance_noise,
           log(std::abs(obs.position.norm()) + 1) /
                   config_.armor_match_conf.ypd_conf.armor_yaw_log_divisor +
               config_.armor_match_conf.ypd_conf.basic_armor_yaw_noise},
      };
      Eigen::Matrix4d R = R_dig.asDiagonal();
      Eigen::Matrix4d S = P + R;
      armors_covs.emplace_back(armor, S);
    }
    armors_covs_vec.emplace_back(armors_covs);
  }
  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
      matched_armors;
  std::array<std::vector<std::pair<double, ArmorPositionRollPitchYawPoints>>, 3>
      index_vector_array;
  for (std::size_t i = 0;
       i < obs_armors_camera.size() && i < obs_armors_odom.size(); ++i) {
    const auto &obs = obs_armors_odom.at(i);
    auto result = Target::matchArmor(armors_covs_vec.at(i), obs);
    if (result.empty())
      continue;
    // NOTE: 在此处从马氏距离更新NIS失败队列
    this->updateNisFailureDeque(result.front().distance);
    if (result.front().distance <
        config_.armor_match_conf.max_match_mahalanobis_distance) {
      index_vector_array.at(static_cast<int>(result.front().index))
          .emplace_back(result.front().distance, obs_armors_camera.at(i));
    }
  }
  int idx = 0;
  for (auto &cost_armor_vec : index_vector_array) {
    std::ranges::sort(
        cost_armor_vec, std::less<>{},
        &std::pair<double, ArmorPositionRollPitchYawPoints>::first);
    if (!cost_armor_vec.empty())
      matched_armors.emplace_back(std::pair{cost_armor_vec.front().second,
                                            static_cast<ArmorIndex>(idx)});
    idx++;
  }
  return matched_armors;
}

auto_aim::OutpostTargetState auto_aim::OutpostTarget::getTargetStateFromArmor(
    const ArmorPositionYaw &armor) const {
  auto armor_x = armor.position.x();
  auto armor_y = armor.position.y();
  auto center_x =
      armor_x + config_.default_radius * std::cos(armor.yaw.theta());
  auto center_y =
      armor_y + config_.default_radius * std::sin(armor.yaw.theta());
  auto dz = armor.position.z();
  OutpostTargetState state;
  state.type = target_state_.type;
  state.center_position = Eigen::Vector3d{center_x, center_y, 0};
  state.center_yaw = armor.yaw.theta();
  state.radius = config_.default_radius;
  state.dz_0 = dz;
  state.dz_1 = dz;
  state.dz_2 = dz;
  return state;
}

void auto_aim::OutpostTarget::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const TargetState &target_state, std::uint64_t k, double dt) const {
  values.insert(X(k), target_state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(target_state.center_yaw));
  values.insert(V(k), target_state.center_velocity);
  values.insert(W(k), target_state.center_vyaw);
  if (k == 0) {
    graph.addPrior(X(0), target_state.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.translation_prior_noise.x,
                       config_.translation_prior_noise.y,
                       // NOTE: 前哨站的centerz仅靠先验约束，噪声必须设小
                       config_.translation_prior_noise.z,
                   }));
    graph.addPrior(
        R(0), gtsam::Rot2::fromAngle(target_state.center_yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_prior_noise));
    graph.addPrior(V(0), target_state.center_velocity,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.velocity_prior_noise.x,
                       config_.velocity_prior_noise.y,
                       config_.velocity_prior_noise.z,
                   }));
    graph.addPrior(
        W(0), target_state.center_vyaw,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_prior_noise));
  } else {
    graph.add(TranslationFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.translation_factor_noise.x,
            config_.translation_factor_noise.y,
            config_.translation_factor_noise.z,
        }),
        X(k - 1),
        V(k - 1),
        X(k),
        dt,
    });
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(k - 1),
        W(k - 1),
        R(k),
        dt,
    });
    graph.add(VelocityFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.velocity_factor_noise.x,
            config_.velocity_factor_noise.y,
            config_.velocity_factor_noise.z,
        }),
        V(k - 1),
        V(k),
    });
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_factor_noise),
        W(k - 1),
        W(k),
    });
  }
  LOG_TRACE_L1(logger_,
               "[Target {}]: add motion values factors. k = {}, dt = {}.",
               rfl::enum_to_string(target_state_.type), k, dt);
}

void auto_aim::OutpostTarget::addArmorReprojValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    gtsam::Key armor_pose_key, const ArmorPositionRollPitchYawPoints &armor,
    std::uint64_t k) const {
  values.insert(armor_pose_key,
                gtsam::Pose3{gtsam::Rot3{armor.getRotation()}, armor.position});
  for (auto position : std::array{
           types::ArmorPointPosition::LeftBottom,
           types::ArmorPointPosition::LeftTop,
           types::ArmorPointPosition::RightTop,
           types::ArmorPointPosition::RightBottom,
       }) {
    graph.add(ArmorReprojFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2{
            config_.armor_observation_noise.pixel_error.x,
            config_.armor_observation_noise.pixel_error.y,
        }),
        armor_pose_key,
        camera_matrix_,
        distortion_coefficients_,
        target_state_.type,
        position,
        armor.points.at(static_cast<int>(position)),
    });
  }
}

void auto_aim::OutpostTarget::addArmorValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
        &armors_indexs,
    const Eigen::Isometry3d &T, std::uint64_t k, double dz_0) const {
  auto default_radius = tools::logisticInverse(
      config_.default_radius, config_.radius_min, config_.radius_max);
  if (k == 0) {
    values.insert(A(0), default_radius);
    values.insert(Z(0), dz_0);
    values.insert(Z(1), dz_0);
    values.insert(Z(2), dz_0);
    graph.addPrior(
        A(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        Z(0), dz_0,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.dz_prior_noise));
    graph.addPrior(
        Z(1), dz_0,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.dz_prior_noise));
    graph.addPrior(
        Z(2), dz_0,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.dz_prior_noise));
  }
  for (const auto &[armor, index] : armors_indexs) {
    auto armor_pose_key = getArmorPoseKeyFromIndex(index, k);
    addArmorReprojValuesFactors(values, graph, armor_pose_key, armor, k);
    graph.add(ArmorRadiusDZFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
            config_.armor_observation_noise.tangential_error_m,
            config_.armor_observation_noise.radial_error_m,
            config_.armor_observation_noise.height_error_m,
            config_.armor_observation_noise.yaw_error_rad,
        }),
        armor_pose_key,
        A(0),
        Z(static_cast<int>(index)),
        R(k),
        X(k),
        T,
        index,
        config_.radius_min,
        config_.radius_max,
        3,
    });
  }
  LOG_TRACE_L1(logger_, "[Target {}]: add {} armor factors. k = {}.",
               rfl::enum_to_string(target_state_.type), armors_indexs.size(),
               k);
}

std::pair<auto_aim::OutpostTargetState, auto_aim::TrackState::State>
auto_aim::OutpostTarget::update(
    const std::vector<auto_aim::ArmorPositionRollPitchYawPoints> &armors,
    double dt, const Eigen::Isometry3d &T) const {
  auto dt_tracking_to_update =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          track_state_.stamp_last_update - track_state_.stamp_last_tracking)
          .count();
  if (track_state_.state != TrackState::State::LOST &&
      dt + dt_tracking_to_update > config_.lost_threshold_sec) {
    LOG_INFO(logger_, "[Target {}]: Time out! dt{}.",
             rfl::enum_to_string(target_state_.type),
             dt + dt_tracking_to_update);
    return {{}, TrackState::State::LOST};
  }
  if (auto nis_fail_opt = nisFailured();
      nis_fail_opt.has_value() && nis_fail_opt.value()) {
    LOG_INFO(logger_, "[Target {}]: Nis failure!.",
             rfl::enum_to_string(target_state_.type));
    return {{}, TrackState::State::LOST};
  }
  auto obs_armors_odom = armors;
  for (auto &armor : obs_armors_odom) {
    Eigen::Isometry3d armor_pose_camera{Eigen::Isometry3d::Identity()};
    armor_pose_camera.pretranslate(armor.position);
    armor_pose_camera.rotate(armor.getRotation());
    auto armor_pose_odom = T * armor_pose_camera;
    auto rpy = tools::rotationMatrixToRPY(armor_pose_odom.rotation());
    armor.position = armor_pose_odom.translation();
    armor.roll = rpy.x();
    armor.pitch = rpy.y();
    armor.yaw = gtsam::Rot2::fromAngle(rpy.z());
  }

  auto target_state = target_state_.predict(dt);
  if (track_state_.state == TrackState::State::LOST) {
    if (obs_armors_odom.empty())
      return {{}, TrackState::State::LOST};
    else
      target_state = getTargetStateFromArmor(obs_armors_odom.front());
  }
  if (std::abs(target_state.center_vyaw) > config_.huge_vyaw_reset_thres) {
    LOG_INFO(logger_, "[Target {}]: Abnormal vyaw {}!",
             rfl::enum_to_string(target_state.type), target_state.center_vyaw);
    return {{}, TrackState::State::LOST};
  }
  // XXX: 这里似乎没有考虑到冷启动过程中的临时丢失
  if (track_state_.k < config_.first_update_batch_size &&
      !obs_armors_odom.empty()) {
    target_state = getTargetStateFromArmor(obs_armors_odom.front());
  }

  auto matched_armors = matchArmors(target_state, dt, armors, obs_armors_odom);
  if (matched_armors.size() < obs_armors_odom.size()) {
    LOG_DEBUG(logger_, "[Target {}]: Miss match {} armors! k = {}.",
              rfl::enum_to_string(target_state.type),
              obs_armors_odom.size() - matched_armors.size(), track_state_.k);
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  if (track_state_.k <= config_.first_update_batch_size) {
    if (track_state_.k == 0) {
      this->initial_values_ = gtsam::Values{};
      this->initial_graph_ = gtsam::NonlinearFactorGraph{};
    }
    values = this->initial_values_;
    graph = this->initial_graph_;
  }
  addMotionValuesFactors(values, graph, target_state, track_state_.k, dt);
  addArmorValuesFactors(values, graph, matched_armors, T, track_state_.k,
                        target_state.dz_0);

  // NOTE: 冷启动时先不更新到isam2里(避免在缺少约束前几帧优化爆掉)
  if (track_state_.k < config_.first_update_batch_size) {
    this->initial_values_ = values;
    this->initial_graph_ = graph;
    return {target_state, matched_armors.empty() ? TrackState::State::TEMPLOST
                                                 : TrackState::State::TRACKING};
  }

  try {
    this->isam2_.update(graph, values);
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_velocity =
        isam2_.calculateEstimate<gtsam::Vector3>(V(track_state_.k));
    target_state.center_yaw =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vyaw =
        isam2_.calculateEstimate<double>(W(track_state_.k));
    target_state.radius =
        tools::logisticFunction(isam2_.calculateEstimate<double>(A(0)),
                                config_.radius_min, config_.radius_max);
    target_state.dz_0 = isam2_.calculateEstimate<double>(Z(0));
    target_state.dz_1 = isam2_.calculateEstimate<double>(Z(1));
    target_state.dz_2 = isam2_.calculateEstimate<double>(Z(2));
    // NOTE: isam2计算联合协方差的jointMarginalCovariance方法还在feature分支上
    // 先用协方差凑合一下吧，等一手gtsam更新
    this->X_cov_ = isam2_.marginalCovariance(X(track_state_.k));
    this->V_cov_ = isam2_.marginalCovariance(V(track_state_.k));
    this->R_cov_ = isam2_.marginalCovariance(R(track_state_.k))(0, 0);
    this->W_cov_ = isam2_.marginalCovariance(W(track_state_.k))(0, 0);
    return {target_state, matched_armors.empty() ? TrackState::State::TEMPLOST
                                                 : TrackState::State::TRACKING};
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "[Target {}]: {}\ncurrent k: {}.",
              rfl::enum_to_string(target_state.type), e.what(), track_state_.k);
    return {{}, TrackState::State::LOST};
  }
}
