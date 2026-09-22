#include "rune_target.hpp"
#include "tasks/auto_buff/rune_track/motion_model.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/dta_utils.hpp"
#include "tasks/base/web.hpp"
#include "utils/common/type_common.hpp"
#include "utils/logger.hpp"
#include "utils/utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <opencv2/core/types.hpp>
#include <optional>
#include <tuple>
#include <vector>
namespace awakening::auto_buff {
using namespace motion_model;

namespace {
    template<int N>
    Eigen::Matrix<double, N, N> diagonal_covariance(double value) {
        return Eigen::Matrix<double, N, N>::Identity() * value;
    }

    FanBladeVecZ fan_blade_observation(const RuneFanBladeWithR& fan) {
        FanBladeVecZ z;
        constexpr std::array indices {
            RuneFanBladeWithR::PointsIndex::TOP,
            RuneFanBladeWithR::PointsIndex::LEFT,
            RuneFanBladeWithR::PointsIndex::BOTTOM,
            RuneFanBladeWithR::PointsIndex::RIGHT,
        };
        for (std::size_t i = 0; i < indices.size(); ++i) {
            z[2 * i] = fan.points[indices[i]].x;
            z[2 * i + 1] = fan.points[indices[i]].y;
        }
        return z;
    }

    FanTargetVecZ fan_target_observation(const RuneFanTarget& fan) {
        FanTargetVecZ z;
        constexpr std::array indices {
            RuneFanTarget::PointsIndex::LT,     RuneFanTarget::PointsIndex::LB,
            RuneFanTarget::PointsIndex::RB,     RuneFanTarget::PointsIndex::RT,
            RuneFanTarget::PointsIndex::CENTER,
        };
        for (std::size_t i = 0; i < indices.size(); ++i) {
            z[2 * i] = fan.key_points[indices[i]].x;
            z[2 * i + 1] = fan.key_points[indices[i]].y;
        }
        return z;
    }

    template<class Iterator>
    cv::Point2f average_point(Iterator begin, Iterator end) {
        const auto count = std::distance(begin, end);
        const auto sum =
            std::accumulate(begin, end, cv::Point2f {}, [](const auto& a, const auto& b) {
                return a + b;
            });
        return sum * (1.0 / count);
    }

    template<typename Fan, typename Pnp>
    std::vector<std::pair<int, Fan>> match_fans_by_ypd(
        const RuneTarget& target,
        std::vector<Fan>& fans,
        const TimePoint& timestamp,
        Pnp&& pnp
    ) noexcept {
        constexpr double MAX_COST = 1e9;
        std::vector<std::pair<int, Fan>> result;
        const int n_obs = static_cast<int>(fans.size());
        std::vector<std::vector<double>> cost(n_obs, std::vector<double>(FAN_NUM, MAX_COST + 1));
        std::vector<YPDVecZ> meas_list(n_obs);
        for (int obs = 0; obs < n_obs; ++obs) {
            pnp(fans[obs]);
            meas_list[obs] = target.get_ypdmeasurement(fans[obs].pose);
        }

        auto pred_state = target.get_target_state();
        pred_state.predict(timestamp, target.voter);
        for (int obs = 0; obs < n_obs; ++obs) {
            bool in_gate = false;
            double min_d2 = std::numeric_limits<double>::max();
            for (int id = 0; id < FAN_NUM; ++id) {
                YPDMeasure measure { .ctx = { .id = id } };
                YPDVecZ z_pred;
                measure.h(pred_state.x, z_pred);

                YPDVecZ nu = meas_list[obs] - z_pred;
                nu[idx::YPD_Y] = angles::normalize_angle(nu[idx::YPD_Y]);
                nu[idx::YPD_P] = angles::normalize_angle(nu[idx::YPD_P]);
                nu[idx::ROT_YAW] = angles::normalize_angle(nu[idx::ROT_YAW]);
                nu[idx::ROT_ROLL] = angles::normalize_angle(nu[idx::ROT_ROLL]);
                auto R = target.ypdmeasurement_covariance(z_pred);
                const double d2 = nu.transpose() * R.ldlt().solve(nu);

                if (std::isfinite(d2) && d2 < target.cfg.match_gate) {
                    cost[obs][id] = d2;
                    in_gate = true;
                }
                min_d2 = std::min(min_d2, d2);
            }
            if (!in_gate) {
                AWAKENING_WARN("match out of gate min d2: {}", min_d2);
            }
        }

        for (auto [obs, id]: dta_utils::greedy_match(cost, n_obs, FAN_NUM, MAX_COST)) {
            result.emplace_back(id, fans[obs]);
        }
        return result;
    }
} // namespace

void RuneTarget::write_log() {
    Web::write_log("rune_target", [&](auto& j) {
        j["timestamp"] =
            static_cast<int>(std::chrono::duration<double>(last_update.time_since_epoch()).count());
        j["track_state"] = TrackState::string_by_state(track_state.tracker_state);
        auto& j_target_state = j["target_state"];
        j_target_state["cx"] = Web::val(target_state.pos().x());
        j_target_state["cy"] = Web::val(target_state.pos().y());
        j_target_state["cz"] = Web::val(target_state.pos().z());
        j_target_state["yaw"] = Web::val(target_state.yaw());
        j_target_state["roll"] = Web::val(target_state.roll());
        j_target_state["v_roll"] = Web::val(target_state.v_roll(voter));
        j_target_state["a"] = Web::val(target_state.a());
        j_target_state["w"] = Web::val(target_state.w());
        j_target_state["tau"] = Web::val(target_state.tau());
        j_target_state["visible"] = fan_wc.to_str();
        j_target_state["voter"] = voter.to_str();
    });
}
bool RuneTarget::reset(
    RuneDetection& d,
    const RuneTrackerCfg& c,
    const TimePoint& timestamp,
    int frame_id,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom
) {
    cfg = c;
    std::optional<ISO3> pose;
    if (!d.fan_blades.empty()) {
        fan_pnp(d.fan_blades.front(), camera_info, camera_cv_in_odom, true);
        pose = d.fan_blades.front().pose;
    } else if (!d.fan_targets.empty() && !d.rune_rs.empty()) {
        const auto closest = std::min_element(
            d.rune_rs.begin(),
            d.rune_rs.end(),
            [&](const RuneR& lhs, const RuneR& rhs) {
                return utils::calculate_distance_to_img_center(
                           lhs.rr.center,
                           camera_info.camera_matrix
                       )
                    < utils::calculate_distance_to_img_center(
                           rhs.rr.center,
                           camera_info.camera_matrix
                    );
            }
        );
        if (closest != d.rune_rs.end()) {
            closest->laji = false;
            fan_target_pnp(
                d.fan_targets.front(),
                closest->rr.center,
                camera_info,
                camera_cv_in_odom,
                true
            );
            pose = d.fan_targets.front().pose;
        }
    }
    if (!pose) {
        return false;
    }
    r_ctx = {
        .camera_cv_in_odom = camera_cv_in_odom,
        .camera_info = camera_info.clone(),
    };
    fan_ctx = {
        .id = 0,
        .camera_cv_in_odom = camera_cv_in_odom,
        .camera_info = camera_info.clone(),
    };
    fan_target_ctx = {
        .id = 0,
        .camera_cv_in_odom = camera_cv_in_odom,
        .camera_info = camera_info.clone(),
    };
    Eigen::DiagonalMatrix<double, X_N> p0;
    p0.diagonal().setZero();
    p0.diagonal()[idx::CX] = p0.diagonal()[idx::CY] = p0.diagonal()[idx::CZ] = 1;
    p0.diagonal()[idx::ROLL] = p0.diagonal()[idx::YAW] = 1;
    p0.diagonal()[idx::V_ROLL] = 100;
    p0.diagonal()[idx::TAU] = 100;
    p0.diagonal()[idx::A] = 1;
    p0.diagonal()[idx::W] = 1;
    const auto u_q = [] { return Eigen::Matrix<double, X_N, X_N>::Zero(); };
    const auto inject = [](const auto& delta, auto& nominal) { inject_state(delta, nominal); };
    const auto box_minus = [](const auto& nominal, const auto& value, auto& delta) {
        box_minus_state(nominal, value, delta);
    };

    voter.reset(timestamp);
    esekf = ESEKF(Predict { .dt = 0.005, .voter = voter }, u_q, inject, box_minus, p0);

    esekf->set_iteration_num(cfg.esekf_iter_num);

    const auto pos = pose->translation();
    const auto rpy = utils::matrix2rpy<double>(pose->linear());
    double a_guess = (A_LOWER + A_UPPER) / 2.0;
    double w_guess = (W_LOWER + W_UPPER) / 2.0;
    double tau = 0;
    if (std::chrono::duration<double>(timestamp - last_update).count() < cfg.big_args_continue_time)
    {
        a_guess = target_state.a();
        w_guess = target_state.w();
        tau = target_state.x[idx::TAU]
            + std::chrono::duration<double>(timestamp - target_state.timestamp).count();
    }
    target_state.x = Eigen::VectorXd::Zero(X_N);
    target_state.set_pos(pos);
    target_state.x[idx::ROLL] = rpy[0];
    target_state.x[idx::YAW] = rpy[2];
    target_state.x[idx::A] = a_guess;
    target_state.x[idx::W] = w_guess;
    target_state.x[idx::TAU] = tau;
    target_state.timestamp = timestamp;
    target_state.frame_id = frame_id;
    esekf->set_state(target_state.x);
    last_update = timestamp;
    is_inited = true;
    track_state.reset();
    this_id = GLOBAL_ID++;
    fan_wc.reset();
    fan_wc.is_visible[0] = true;
    return true;
}
void RuneTarget::fan_pnp(
    RuneFanBladeWithR& r,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom,
    bool in_r
) noexcept {
    auto key_points = r.points;
    // std::vector<cv::Mat> rvecs;
    // std::vector<cv::Mat> tvecs;
    // if (!cv::solvePnPGeneric(
    //         in_r ? RuneFanBladeWithR::Point3DRZERO<cv::Point3f>::build()
    //              : RuneFanBladeWithR::Point3DTargetCenterZERO<cv::Point3f>::build(),
    //         key_points,
    //         camera_info.camera_matrix,
    //         camera_info.distortion_coefficients,
    //         rvecs,
    //         tvecs,
    //         false,
    //         cv::SOLVEPNP_IPPE,
    //         cv::noArray(),
    //         cv::noArray()
    //     ))
    // {
    //     return;
    // }

    // bool has_valid = false;
    // for (size_t i = 0; i < rvecs.size(); ++i) {
    //     cv::Mat R_cv;
    //     cv::Rodrigues(rvecs[i], R_cv);
    //     Mat3 R_eigen;
    //     cv::cv2eigen(R_cv, R_eigen);
    //     Vec3 axis_x = R_eigen.col(0);
    //     Vec3 t_eigen;
    //     cv::cv2eigen(tvecs[i], t_eigen);
    //     Vec3 front_normal = -axis_x;
    //     if (front_normal.dot(-t_eigen) > 0)
    //     { //选择正面朝向相机，这里重投影误差已经进行过排序，所以直接break
    //         r.pose.translation() = t_eigen;
    //         r.pose.linear() = R_eigen;
    //         has_valid = true;
    //         break;
    //     }
    // }
    r.pose = utils::solve_pnp(
        key_points,
        in_r ? RuneFanBladeWithR::Point3DRZERO<cv::Point3f>::build()
             : RuneFanBladeWithR::Point3DTargetCenterZERO<cv::Point3f>::build(),
        camera_info.camera_matrix,
        camera_info.distortion_coefficients
    );

    r.pose = camera_cv_in_odom * r.pose;
}
void RuneTarget::fan_target_pnp(
    RuneFanTarget& a,
    const cv::Point2f& r,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom,
    bool in_r
) noexcept {
    a.sort_corners(r);
    auto key_points = a.key_points;
    // std::vector<cv::Mat> rvecs;
    // std::vector<cv::Mat> tvecs;
    // if (!cv::solvePnPGeneric(
    //         in_r ? RuneFanTarget::Point3DRZERO<cv::Point3f>::build_no_r()
    //              : RuneFanTarget::Point3DTargetCenterZERO<cv::Point3f>::build_no_r(),
    //         key_points,
    //         camera_info.camera_matrix,
    //         camera_info.distortion_coefficients,
    //         rvecs,
    //         tvecs,
    //         false,
    //         cv::SOLVEPNP_IPPE,
    //         cv::noArray(),
    //         cv::noArray()
    //     ))
    // {
    //     return;
    // }

    // bool has_valid = false;
    // for (size_t i = 0; i < rvecs.size(); ++i) {
    //     cv::Mat R_cv;
    //     cv::Rodrigues(rvecs[i], R_cv);
    //     Mat3 R_eigen;
    //     cv::cv2eigen(R_cv, R_eigen);
    //     Vec3 axis_x = R_eigen.col(0);
    //     Vec3 t_eigen;
    //     cv::cv2eigen(tvecs[i], t_eigen);
    //     Vec3 front_normal = -axis_x;
    //     if (front_normal.dot(-t_eigen) > 0)
    //     { //选择正面朝向相机，这里重投影误差已经进行过排序，所以直接break
    //         a.pose.translation() = t_eigen;
    //         a.pose.linear() = R_eigen;
    //         has_valid = true;
    //         break;
    //     }
    // }
    a.pose = utils::solve_pnp(
        key_points,
        in_r ? RuneFanTarget::Point3DRZERO<cv::Point3f>::build_no_r()
             : RuneFanTarget::Point3DTargetCenterZERO<cv::Point3f>::build_no_r(),
        camera_info.camera_matrix,
        camera_info.distortion_coefficients
    );
    a.pose = camera_cv_in_odom * a.pose;
}
[[nodiscard]] Eigen::Matrix<double, motion_model::X_N, motion_model::X_N>
RuneTarget::process_noise(double dt) const noexcept {
    Eigen::Matrix<double, X_N, X_N> q;
    q.setZero();
    auto dt2 = dt * dt;
    q(idx::CX, idx::CX) = dt * cfg.q_xyz.x();
    q(idx::CY, idx::CY) = dt * cfg.q_xyz.y();
    q(idx::CZ, idx::CZ) = dt * cfg.q_xyz.z();
    q(idx::YAW, idx::YAW) = dt * cfg.q_yaw;

    utils::fill_constant_accel_noise(q, idx::ROLL, idx::V_ROLL, cfg.q_roll, dt);
    q(idx::A, idx::A) = dt * cfg.q_a;
    q(idx::W, idx::W) = dt * cfg.q_w;
    q(idx::TAU, idx::TAU) = dt * cfg.q_tau;

    return q;
}
[[nodiscard]] Eigen::Matrix<double, motion_model::YPDZ_N, motion_model::YPDZ_N>
RuneTarget::ypdmeasurement_covariance(const Eigen::Matrix<double, motion_model::YPDZ_N, 1>& z
) const noexcept {
    Eigen::Matrix<double, YPDZ_N, YPDZ_N> r;

    r.setZero(); //copy下sp_vision_25 这个参数不用在观测，差不多就行
    r(idx::YPD_Y, idx::YPD_Y) = 4e-3;
    r(idx::YPD_P, idx::YPD_P) = 4e-3;
    r(idx::YPD_D, idx::YPD_D) = 0.5;
    r(idx::ROT_YAW, idx::ROT_YAW) = 0.1;
    r(idx::ROT_ROLL, idx::ROT_ROLL) = 0.05;
    return r;
}
[[nodiscard]] Eigen::Matrix<double, motion_model::YPDZ_N, 1>
RuneTarget::get_ypdmeasurement(const ISO3& pose) const noexcept {
    Eigen::Matrix<double, YPDZ_N, 1> z;
    const Vec3 position = pose.translation();
    const double xy_distance = position.head<2>().norm();
    z[idx::YPD_Y] = std::atan2(position.y(), position.x());
    z[idx::YPD_P] = std::atan2(position.z(), xy_distance);
    z[idx::YPD_D] = position.norm();
    const auto rpy = utils::matrix2rpy<double>(pose.linear());
    z[idx::ROT_YAW] = rpy[2];
    z[idx::ROT_ROLL] = rpy[0];
    return z;
}
void RuneTarget::predict_ekf(const TimePoint& timestamp) {
    if (!esekf) {
        throw std::runtime_error("ESEKF is not initialized");
    }
    auto dt = std::chrono::duration<double>(timestamp - target_state.timestamp).count();
    esekf->set_predict_func(Predict { .dt = dt, .voter = voter });
    esekf->set_update_Q([&]() { return process_noise(dt); });
    target_state.x = esekf->predict();
    target_state.timestamp = timestamp;
    this_id = GLOBAL_ID++;
}
std::optional<std::pair<bool, cv::Point2f>> RuneTarget::match_r(
    const std::vector<std::pair<int, RuneFanBladeWithR>>& matched_fans,
    std::vector<RuneR>& rs,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom
) {
    if (rs.empty()) {
        return std::nullopt;
    }
    auto pred_state = target_state;
    pred_state.predict(timestamp, voter);
    RVecZ r_prediction;
    auto r_measure_ctx = r_ctx;
    r_measure_ctx.camera_cv_in_odom = camera_cv_in_odom;
    RMeasure r_measure { .ctx = r_measure_ctx };
    r_measure.h(pred_state.x, r_prediction);
    FanBladeVecZ fan_prediction;
    auto fan_measure_ctx = fan_ctx;
    fan_measure_ctx.camera_cv_in_odom = camera_cv_in_odom;
    fan_measure_ctx.id = 0;
    FanBladeMeasure fan_measure { .ctx = fan_measure_ctx };
    fan_measure.h(pred_state.x, fan_prediction);
    std::vector<cv::Point2f> r_vec;
    std::vector<double> fan_hand_length_vec;
    for (const auto& match: matched_fans) {
        const auto& fan = match.second;
        r_vec.push_back(fan.points[RuneFanBladeWithR::PointsIndex::R]);
        fan_hand_length_vec.push_back(cv::norm(
            fan.points[RuneFanBladeWithR::PointsIndex::BOTTOM]
            - fan.points[RuneFanBladeWithR::PointsIndex::R]
        ));
    }
    r_vec.emplace_back(r_prediction[idx::R_X], r_prediction[idx::R_Y]);
    fan_hand_length_vec.push_back(cv::norm(
        cv::Point2f(fan_prediction[idx::BOTTOM_X], fan_prediction[idx::BOTTOM_Y])
        - cv::Point2f(r_prediction[idx::R_X], r_prediction[idx::R_Y])
    ));
    auto avg_r = average_point(r_vec.begin(), r_vec.end());
    const double avg_hand_length =
        std::accumulate(fan_hand_length_vec.begin(), fan_hand_length_vec.end(), 0.0)
        / fan_hand_length_vec.size();
    int best_id = -1;
    double min_cost = std::numeric_limits<double>::max();
    // std::cout<<avg_hand_length<<std::endl;
    for (size_t i = 0; i < rs.size(); ++i) {
        const double error = cv::norm(rs[i].rr.center - avg_r);
        if (error > avg_hand_length * 0.2) {
            continue;
        }
        if (error < min_cost) {
            min_cost = error;
            best_id = i;
        }
    }
    if (best_id != -1) {
        rs[best_id].laji = false;
        return std::make_pair(true, rs[best_id].rr.center);
    } else if (!matched_fans.empty() && r_vec.size() > 1) {
        return std::make_pair(false, average_point(r_vec.begin(), r_vec.end() - 1));
    }
    return std::nullopt;
}
std::tuple<int, int> RuneTarget::update(
    const std::vector<std::pair<int, RuneFanBladeWithR>>& matched_fans,
    const std::vector<std::pair<int, RuneFanTarget>>& matched_fan_targets,
    const std::optional<std::pair<bool, cv::Point2f>>& r,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom
) {
    std::vector<std::shared_ptr<ESEKF::ObsBase>> obs;
    auto pred_state = target_state;
    pred_state.predict(timestamp, voter);
    FanBladeVecZ fan_prediction;
    auto fan_measure_ctx = fan_ctx;
    fan_measure_ctx.camera_cv_in_odom = camera_cv_in_odom;
    fan_measure_ctx.id = 0;
    FanBladeMeasure fan_measure { .ctx = fan_measure_ctx };
    fan_measure.h(pred_state.x, fan_prediction);
    RVecZ r_prediction;
    auto r_measure_ctx = r_ctx;
    r_measure_ctx.camera_cv_in_odom = camera_cv_in_odom;
    RMeasure r_measure { .ctx = r_measure_ctx };
    r_measure.h(pred_state.x, r_prediction);
    double fan_hand_length = cv::norm(
        cv::Point2f(fan_prediction[idx::BOTTOM_X], fan_prediction[idx::BOTTOM_Y])
        - cv::Point2f(r_prediction[idx::R_X], r_prediction[idx::R_Y])
    );
    if (r) {
        RVecZ z(r->second.x, r->second.y);
        auto ctx = r_ctx;
        ctx.camera_cv_in_odom = camera_cv_in_odom;
        RMeasure measure { .ctx = ctx };
        const auto r_u_r = [&](const auto&) {
            auto sigma_ratio = r->first ? cfg.r_sigma_pix_by_hand_length_ratio_cv
                                        : cfg.r_sigma_pix_by_hand_length_ratio_net;
            auto sigma = sigma_ratio * fan_hand_length;
            return diagonal_covariance<RZ_N>(sigma * sigma / 2.0);
        };
        const auto residual = [](const auto& z_pred, const auto& z) { return z - z_pred; };
        auto tmp_ekf = esekf.value();
        auto tmp_state = target_state;
        tmp_state.x = tmp_ekf.update(z, measure, r_u_r, residual);
        if ((tmp_state.pos() - target_state.pos()).norm() < 1.0) {
            obs.push_back(esekf->make_obs(z, measure, r_u_r, residual));
        } else {
            AWAKENING_WARN("FUCK r");
        }
    }

    const auto fan_u_r = [&](const auto&) {
        auto sigma_ratio = cfg.r_sigma_pix_by_hand_length_ratio_net;
        auto sigma = sigma_ratio * fan_hand_length;
        return diagonal_covariance<FanBladeZ_N>(sigma * sigma / 2.0);
    };
    const auto residual = [](const auto& z_pred, const auto& z) { return z - z_pred; };
    if (!matched_fans.empty() || !matched_fan_targets.empty()) {
        fan_wc.reset();
    }
    for (const auto& [id, fan]: matched_fans) {
        fan_wc.update(id, timestamp);
        auto ctx = fan_ctx;
        ctx.id = id;
        ctx.camera_cv_in_odom = camera_cv_in_odom;
        FanBladeMeasure measure { .ctx = ctx };
        obs.push_back(esekf->make_obs(fan_blade_observation(fan), measure, fan_u_r, residual));
    }
    const auto fan_target_u_r = [&](const auto&) {
        auto sigma_ratio = cfg.r_sigma_pix_by_hand_length_ratio_cv;
        auto sigma = sigma_ratio * fan_hand_length;
        return diagonal_covariance<FanTargetZ_N>(sigma * sigma / 2.0);
    };
    for (const auto& [id, fan]: matched_fan_targets) {
        fan_wc.update(id, timestamp);
        auto ctx = fan_target_ctx;
        ctx.id = id;
        ctx.camera_cv_in_odom = camera_cv_in_odom;
        FanTargetMeasure measure { .ctx = ctx };
        obs.push_back(
            esekf->make_obs(fan_target_observation(fan), measure, fan_target_u_r, residual)
        );
    }
    if (!obs.empty()) {
        target_state.x = esekf->update_multi(obs);
        target_state.timestamp = timestamp;
        last_update = timestamp;
        this_id = GLOBAL_ID++;
    }
    const int match_fan_num = matched_fans.size() + matched_fan_targets.size();
    if (match_fan_num > 0) {
        voter.update(target_state.roll(), cfg.voter_state_need_count);
    }
    if (matched_fans.size() >= 2 || matched_fan_targets.size() >= 2) {
        voter.double_detect_count++;
        if (voter.double_detect_count > cfg.voter_mode_need_count) {
            voter.mode = Voter::Big;
        }
    }
    return { match_fan_num, r ? 1 : 0 };
}
std::vector<std::pair<int, RuneFanBladeWithR>> RuneTarget::match_fan(
    std::vector<RuneFanBladeWithR>& fans,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom
) const noexcept {
    return match_fans_by_ypd(*this, fans, timestamp, [&](RuneFanBladeWithR& fan) {
        fan_pnp(fan, camera_info, camera_cv_in_odom, true);
    });
}
std::vector<std::pair<int, RuneFanTarget>> RuneTarget::match_fan_target(
    std::vector<RuneFanTarget>& fans,
    const std::optional<std::pair<bool, cv::Point2f>>& r,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom
) const noexcept {
    cv::Point2f r_tag;
    // if (!r) {
    auto pred_state = target_state;
    pred_state.predict(timestamp, voter);
    RVecZ r_prediction;
    auto r_measure_ctx = r_ctx;
    r_measure_ctx.camera_cv_in_odom = camera_cv_in_odom;
    RMeasure r_measure { .ctx = r_measure_ctx };
    r_measure.h(pred_state.x, r_prediction);
    r_tag = cv::Point2f(r_prediction[idx::R_X], r_prediction[idx::R_Y]);
    // } else {
    //     r_tag = r->second;
    // }
    return match_fans_by_ypd(*this, fans, timestamp, [&](RuneFanTarget& fan) {
        fan_target_pnp(fan, r_tag, camera_info, camera_cv_in_odom, true);
    });
}
[[nodiscard]] cv::Rect RuneTarget::get_net_focus_roi(
    const TimePoint& timestamp,
    const ISO3& camera_cv_in_odom,
    const CameraInfo& camera_info,
    const cv::Size& image_size,
    double target_wh_ratio
) const noexcept {
    if (!need_focus()) {
        return cv::Rect(0, 0, image_size.width, image_size.height);
    }

    auto tmp_target_state = target_state;
    tmp_target_state.predict(timestamp, voter);

    constexpr float car_box_half = 1.0f;
    static const std::vector<cv::Point3f> CAR_BOX {
        { 0, car_box_half, -car_box_half },
        { 0, -car_box_half, -car_box_half },
        { 0, -car_box_half, car_box_half },
        { 0, car_box_half, car_box_half },
    };

    auto pos = tmp_target_state.pos();
    ISO3 pose_in_odom = ISO3::Identity();
    pose_in_odom.translation() = pos;
    pose_in_odom.linear() = utils::rpy2matrix(Vec3(0, 0, std::atan2(pos.x(), pos.y())));
    auto pose_in_camera_cv = camera_cv_in_odom.inverse() * pose_in_odom;

    auto pts = utils::reprojection(
        camera_info.camera_matrix,
        camera_info.distortion_coefficients,
        CAR_BOX,
        pose_in_camera_cv
    );
    cv::Rect rect = cv::boundingRect(pts);
    cv::Rect img_rect(0, 0, image_size.width, image_size.height);

    if ((rect & img_rect).area() <= 100) {
        return img_rect;
    }
    rect &= img_rect;

    double rect_w = std::max<double>(rect.width, 1.0);
    double rect_h = std::max<double>(rect.height, 1.0);
    double ratio =
        (std::isfinite(target_wh_ratio) && target_wh_ratio > 0.0) ? target_wh_ratio : 1.0;

    double target_w = rect_w;
    double target_h = rect_h;
    if (target_w / target_h < ratio) {
        target_w = target_h * ratio;
    } else {
        target_h = target_w / ratio;
    }

    double cx = rect.x + rect.width / 2.0;
    double cy = rect.y + rect.height / 2.0;
    cv::Rect ratio_rect(
        static_cast<int>(cx - target_w / 2.0),
        static_cast<int>(cy - target_h / 2.0),
        static_cast<int>(target_w),
        static_cast<int>(target_h)
    );
    ratio_rect &= img_rect;

    double dt = std::chrono::duration<double>(timestamp - last_update).count();
    double lost_dt = cfg.lost_time_thres;
    const double dt_clamped = std::clamp(dt, 0.0, lost_dt);

    int base_side = std::max(ratio_rect.width, ratio_rect.height);
    int max_side = std::max(image_size.width, image_size.height);
    int side = static_cast<int>(base_side + (max_side - base_side) * (dt_clamped / lost_dt));
    if (dt >= lost_dt)
        side = max_side;

    int square_cx = ratio_rect.x + ratio_rect.width / 2;
    int square_cy = ratio_rect.y + ratio_rect.height / 2;
    cv::Rect square(square_cx - side / 2, square_cy - side / 2, side, side);
    square &= img_rect;

    return square;
}
} // namespace awakening::auto_buff
