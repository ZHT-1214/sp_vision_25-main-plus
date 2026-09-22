#pragma once
#include "KalmanHyLib/error_state_extended_kalman_filter.hpp"
#include "angles.h"
#include "tasks/auto_aim/type.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/common.hpp"
#include "utils/common/type_common.hpp"
#include "utils/utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cassert>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <string>
#include <utility>
#include <vector>
namespace awakening::auto_buff::motion_model {

namespace idx {
    enum { CX, CY, CZ, YAW, ROLL, V_ROLL, TAU, A, W, X_N };
    enum { R_X, R_Y, _R_Z_N };
    enum { TOP_X, TOP_Y, LEFT_X, LEFT_Y, BOTTOM_X, BOTTOM_Y, RIGHT_X, RIGHT_Y, _FanBlade_Z_N };
    enum { LT_X, LT_Y, LB_X, LB_Y, RB_X, RB_Y, RT_X, RT_Y, CEN_X, CEN_Y, _FanTarget_Z_N };
    enum { YPD_Y, YPD_P, YPD_D, ROT_YAW, ROT_ROLL, _YPD_Z_N };
} // namespace idx
constexpr int X_N = idx::X_N;
constexpr int RZ_N = idx::_R_Z_N;
constexpr int FanBladeZ_N = idx::_FanBlade_Z_N;
constexpr int FanTargetZ_N = idx::_FanTarget_Z_N;
constexpr int YPDZ_N = idx::_YPD_Z_N;
constexpr double SMALL_SPEED = M_PI / 3.0;
constexpr double AMPLITUDE_SUM = 2.090;
constexpr double A_LOWER = 0.780;
constexpr double A_UPPER = 1.045;
constexpr double W_LOWER = 1.884;
constexpr double W_UPPER = 2.000;

using VecX = Eigen::Matrix<double, X_N, 1>;
using RVecZ = Eigen::Matrix<double, RZ_N, 1>;
using FanBladeVecZ = Eigen::Matrix<double, FanBladeZ_N, 1>;
using FanTargetVecZ = Eigen::Matrix<double, FanTargetZ_N, 1>;
using YPDVecZ = Eigen::Matrix<double, YPDZ_N, 1>;
template<typename T>
inline T normalize_angle(T a) {
    const T two_pi = T(2.0 * M_PI);
    return a - two_pi * ceres::floor((a + T(M_PI)) / two_pi);
}
template<class DeltaVector, class StateVector>
inline void inject_state(const DeltaVector& delta, StateVector& nominal) {
    for (int i = 0; i < X_N; ++i) {
        if (i != idx::YAW && i != idx::ROLL)
            nominal[i] += delta[i];
    }
    nominal[idx::YAW] = normalize_angle(nominal[idx::YAW] + delta[idx::YAW]);
    nominal[idx::ROLL] = normalize_angle(nominal[idx::ROLL] + delta[idx::ROLL]);
}
template<class StateVector, class DeltaVector>
inline void
box_minus_state(const StateVector& nominal, const StateVector& value, DeltaVector& delta) {
    delta = value - nominal;
    delta[idx::YAW] = normalize_angle(value[idx::YAW] - nominal[idx::YAW]);
    delta[idx::ROLL] = normalize_angle(value[idx::ROLL] - nominal[idx::ROLL]);
}

struct Voter {
    enum {
        Collecting,
        Clockwise,
        Counterclockwise,
    } state = Collecting;
    enum { Big, Small } mode = Small;
    void reset(const TimePoint&) {
        *this = {};
    }
    void update(double roll, int need_count) {
        const double diff = angles::normalize_angle(roll - last_state_roll);
        if (std::abs(diff) < 0.05) {
            return;
        }
        if (diff > 0) {
            clock_wise_count++;
        } else {
            clock_wise_count--;
        }
        last_state_roll = roll;
        if (std::abs(clock_wise_count) > need_count) {
            state = clock_wise_count > 0 ? Clockwise : Counterclockwise;
        } else {
            state = Collecting;
        }
    }
    std::string to_str() const {
        std::string str;
        str += "State: ";
        switch (state) {
            case Collecting:
                str += "Collecting";
                break;
            case Clockwise:
                str += "Clockwise";
                break;
            case Counterclockwise:
                str += "Counterclockwise";
                break;
        }
        str += ", Mode: ";
        str += (mode == Big ? "Big" : "Small");
        return str;
    }
    int clock_wise_count = 0;
    double last_state_roll = 0.0;
    int double_detect_count = 0;
};
struct Predict {
    double dt { 0.0 };
    Voter voter;

    template<typename T>
    inline void operator()(const T x0[X_N], T x1[X_N]) const {
        assert(x0 != x1);
        std::copy(x0, x0 + X_N, x1);
        T delta_theta_abs;
        T delta_theta;

        x1[idx::TAU] += dt;
        if (voter.mode == Voter::Big) {
            const T a = x0[idx::A];
            const T w = x0[idx::W];
            const T b = T(AMPLITUDE_SUM) - a;
            delta_theta_abs =
                ((a / w) * (ceres::cos(w * x0[idx::TAU]) - ceres::cos(w * x1[idx::TAU])))
                + b * T(dt);
        } else {
            delta_theta_abs = T(SMALL_SPEED) * T(dt);
        }
        if (voter.state == Voter::Collecting) {
            delta_theta = x0[idx::V_ROLL] * T(dt);
        } else if (voter.state == Voter::Clockwise) {
            delta_theta = delta_theta_abs;
        } else {
            delta_theta = -delta_theta_abs;
        }
        x1[idx::ROLL] += delta_theta;
        clamp(x1);
    }

    template<typename T>
    inline void clamp(T x[X_N]) const {
        if (voter.state != Voter::Collecting) {
            x[idx::V_ROLL] = T(0);
        }
        // x[idx::A] = ceres::fmin(x[idx::A], A_UPPER);
        // x[idx::W] = ceres::fmin(x[idx::W], W_UPPER);
        // x[idx::A] = ceres::fmax(x[idx::A], A_LOWER);
        // x[idx::W] = ceres::fmax(x[idx::W], W_LOWER);
        if (x[idx::A] > T(A_UPPER)) {
            x[idx::A] = T(A_UPPER);
        }
        if (x[idx::W] > T(W_UPPER)) {
            x[idx::W] = T(W_UPPER);
        }
        if (x[idx::A] < T(A_LOWER)) {
            x[idx::A] = T(A_LOWER);
        }
        if (x[idx::W] < T(W_LOWER)) {
            x[idx::W] = T(W_LOWER);
        }
    }
    inline void f(const VecX& x0, VecX& x1) const {
        assert(x0.size() == X_N);
        assert(x1.size() == X_N);
        operator()(x0.data(), x1.data());
    }
};

struct RMeasure {
    struct Ctx {
        ISO3 camera_cv_in_odom = ISO3::Identity();
        CameraInfo camera_info;
    } ctx;

    template<typename T>
    inline void operator()(const T x[X_N], T z[RZ_N]) const {
        Eigen::Transform<T, 3, Eigen::Isometry> pose_in_odom =
            Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
        pose_in_odom.translation() << x[idx::CX], x[idx::CY], x[idx::CZ];

        Eigen::Transform<T, 3, Eigen::Isometry> camera_cv_in_odom_jet;
        camera_cv_in_odom_jet.matrix() = ctx.camera_cv_in_odom.matrix().template cast<T>();

        auto pose_in_camera_cv = camera_cv_in_odom_jet.inverse() * pose_in_odom;
        std::vector<Eigen::Matrix<T, 2, 1>> img_pts_jet;
        utils::project_points_jets(
            { cv::Point3f(0, 0, 0) },
            pose_in_camera_cv,
            ctx.camera_info.camera_matrix,
            ctx.camera_info.distortion_coefficients,
            img_pts_jet
        );
        z[idx::R_X] = img_pts_jet[0].x();
        z[idx::R_Y] = img_pts_jet[0].y();
    }

    inline void h(const VecX& x, RVecZ& z) const {
        operator()(x.data(), z.data());
    }
};
template<typename T>
inline Eigen::Transform<T, 3, Eigen::Isometry> rune_pose(const T x[X_N], int id) {
    const T roll = normalize_angle(x[idx::ROLL] + T(id) * T(2.0 * M_PI / FAN_NUM));
    Eigen::Transform<T, 3, Eigen::Isometry> rune_in_odom =
        Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    rune_in_odom.translation() << x[idx::CX], x[idx::CY], x[idx::CZ];
    const T yaw = normalize_angle(ceres::atan2(x[idx::CY], x[idx::CX]));
    Eigen::Quaternion<T> q_yaw_rune_in_odom(Eigen::AngleAxis<T>(yaw, Eigen::Vector3<T>::UnitZ()));
    Eigen::Quaternion<T> q_pitch_rune_in_odom(Eigen::AngleAxis<T>(T(0), Eigen::Vector3<T>::UnitY())
    );
    Eigen::Quaternion<T> q_roll_rune_in_odom(Eigen::AngleAxis<T>(roll, Eigen::Vector3<T>::UnitX()));
    rune_in_odom.linear() =
        (q_yaw_rune_in_odom * q_pitch_rune_in_odom * q_roll_rune_in_odom).toRotationMatrix();
    return rune_in_odom;
}
template<typename T>
inline Eigen::Transform<T, 3, Eigen::Isometry> fan_target_pose(const T x[X_N], int id) {
    Eigen::Transform<T, 3, Eigen::Isometry> fan_target_in_rune =
        Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    fan_target_in_rune.translation() << T(0), T(0), T(RUNE_R2_FAN_TARGET_CENTER);
    return rune_pose(x, id) * fan_target_in_rune;
}

struct FanBladeMeasure {
    struct Ctx {
        int id = 0;
        ISO3 camera_cv_in_odom = ISO3::Identity();
        CameraInfo camera_info;
    } ctx;

    template<typename T>
    inline void operator()(const T x[X_N], T z[FanBladeZ_N]) const {
        auto pose_in_odom = rune_pose(x, ctx.id);
        Eigen::Transform<T, 3, Eigen::Isometry> camera_cv_in_odom_jet;
        camera_cv_in_odom_jet.matrix() = ctx.camera_cv_in_odom.matrix().template cast<T>();

        auto pose_in_camera_cv = camera_cv_in_odom_jet.inverse() * pose_in_odom;
        std::vector<Eigen::Matrix<T, 2, 1>> img_pts_jet;
        utils::project_points_jets(
            RuneFanBladeWithR::Point3DRZERO<cv::Point3f>::build(),
            pose_in_camera_cv,
            ctx.camera_info.camera_matrix,
            ctx.camera_info.distortion_coefficients,
            img_pts_jet
        );
        z[idx::TOP_X] = img_pts_jet[RuneFanBladeWithR::PointsIndex::TOP].x();
        z[idx::TOP_Y] = img_pts_jet[RuneFanBladeWithR::PointsIndex::TOP].y();
        z[idx::LEFT_X] = img_pts_jet[RuneFanBladeWithR::PointsIndex::LEFT].x();
        z[idx::LEFT_Y] = img_pts_jet[RuneFanBladeWithR::PointsIndex::LEFT].y();
        z[idx::BOTTOM_X] = img_pts_jet[RuneFanBladeWithR::PointsIndex::BOTTOM].x();
        z[idx::BOTTOM_Y] = img_pts_jet[RuneFanBladeWithR::PointsIndex::BOTTOM].y();
        z[idx::RIGHT_X] = img_pts_jet[RuneFanBladeWithR::PointsIndex::RIGHT].x();
        z[idx::RIGHT_Y] = img_pts_jet[RuneFanBladeWithR::PointsIndex::RIGHT].y();
    }

    inline void h(const VecX& x, FanBladeVecZ& z) const {
        operator()(x.data(), z.data());
    }
};
struct FanTargetMeasure {
    struct Ctx {
        int id = 0;
        ISO3 camera_cv_in_odom = ISO3::Identity();
        CameraInfo camera_info;
    } ctx;

    template<typename T>
    inline void operator()(const T x[X_N], T z[FanTargetZ_N]) const {
        auto pose_in_odom = rune_pose(x, ctx.id);
        Eigen::Transform<T, 3, Eigen::Isometry> camera_cv_in_odom_jet;
        camera_cv_in_odom_jet.matrix() = ctx.camera_cv_in_odom.matrix().template cast<T>();

        auto pose_in_camera_cv = camera_cv_in_odom_jet.inverse() * pose_in_odom;
        std::vector<Eigen::Matrix<T, 2, 1>> img_pts_jet;
        utils::project_points_jets(
            RuneFanTarget::Point3DRZERO<cv::Point3f>::build_no_r(),
            pose_in_camera_cv,
            ctx.camera_info.camera_matrix,
            ctx.camera_info.distortion_coefficients,
            img_pts_jet
        );
        z[idx::LT_X] = img_pts_jet[RuneFanTarget::PointsIndex::LT].x();
        z[idx::LT_Y] = img_pts_jet[RuneFanTarget::PointsIndex::LT].y();
        z[idx::LB_X] = img_pts_jet[RuneFanTarget::PointsIndex::LB].x();
        z[idx::LB_Y] = img_pts_jet[RuneFanTarget::PointsIndex::LB].y();
        z[idx::RB_X] = img_pts_jet[RuneFanTarget::PointsIndex::RB].x();
        z[idx::RB_Y] = img_pts_jet[RuneFanTarget::PointsIndex::RB].y();
        z[idx::RT_X] = img_pts_jet[RuneFanTarget::PointsIndex::RT].x();
        z[idx::RT_Y] = img_pts_jet[RuneFanTarget::PointsIndex::RT].y();
        z[idx::CEN_X] = img_pts_jet[RuneFanTarget::PointsIndex::CENTER].x();
        z[idx::CEN_Y] = img_pts_jet[RuneFanTarget::PointsIndex::CENTER].y();
    }

    inline void h(const VecX& x, FanTargetVecZ& z) const {
        operator()(x.data(), z.data());
    }
};
struct YPDMeasure {
    struct Ctx {
        int id = 0;
    } ctx;

    template<typename T>
    inline void operator()(const T x[X_N], T z[YPDZ_N]) const {
        const T roll = normalize_angle(x[idx::ROLL] + T(ctx.id) * T(2.0 * M_PI / FAN_NUM));
        Eigen::Transform<T, 3, Eigen::Isometry> pose_in_odom = rune_pose(x, ctx.id);
        T target_x = pose_in_odom.translation().x();
        T target_y = pose_in_odom.translation().y();
        T target_z = pose_in_odom.translation().z();

        T xy_dist = ceres::sqrt(target_x * target_x + target_y * target_y);
        T dist = ceres::sqrt(xy_dist * xy_dist + target_z * target_z);
        z[idx::YPD_Y] = ceres::atan2(target_y, target_x); // yaw
        z[idx::YPD_P] = ceres::atan2(target_z, xy_dist); // pitch
        z[idx::YPD_D] = dist; // distance
        z[idx::ROT_YAW] = x[idx::YAW]; // orientation_yaw
        z[idx::ROT_ROLL] = roll; // orientation_roll
    }

    inline void h(const VecX& x, YPDVecZ& z) const {
        operator()(x.data(), z.data());
    }
};

struct State {
    VecX x;
    TimePoint timestamp;
    int frame_id = 0;
    inline std::vector<ISO3> get_fan_target_pose() const {
        std::vector<ISO3> r;
        r.reserve(FAN_NUM);
        for (int i = 0; i < FAN_NUM; ++i) {
            r.push_back(fan_target_pose(x.data(), i));
        }

        return r;
    }
    inline std::vector<ISO3> get_fan_pose() const {
        std::vector<ISO3> r;
        r.reserve(FAN_NUM);
        for (int i = 0; i < FAN_NUM; ++i) {
            r.push_back(rune_pose(x.data(), i));
        }

        return r;
    }
    inline void predict(const TimePoint& t, const Voter& voter) {
        auto dt = std::chrono::duration<double>(t - timestamp).count();
        predict(dt, voter);
    }
    inline void predict(double dt, const Voter& voter) {
        Predict p { .dt = dt, .voter = voter };
        auto tmp_x = x;
        p.f(tmp_x, x);
        timestamp +=
            std::chrono::duration_cast<TimePoint::duration>(std::chrono::duration<double>(dt));
    }
    void set_pos(const Vec3& p) {
        x[idx::CX] = p.x();
        x[idx::CY] = p.y();
        x[idx::CZ] = p.z();
    }
    Vec3 pos() const {
        return Vec3(x[idx::CX], x[idx::CY], x[idx::CZ]);
    }
    double yaw() const {
        return x[idx::YAW];
    }
    double roll() const {
        return x[idx::ROLL];
    }
    double v_roll(const Voter& voter) const {
        if (voter.state == Voter::Collecting) {
            return x[idx::V_ROLL];
        }
        int dir = voter.state == Voter::Clockwise ? 1 : -1;
        if (voter.mode == Voter::Big) {
            const double a = this->a();
            const double w = this->w();
            return dir * (a * std::sin(w * x[idx::TAU]) + (AMPLITUDE_SUM - a));
        }
        return dir * SMALL_SPEED;
    }
    double a() const {
        return x[idx::A];
    }
    double w() const {
        return x[idx::W];
    }
    double tau() const {
        return x[idx::TAU];
    }
};

using ESEKF = kalman_hybird_lib::ErrorStateEKF<X_N, Predict>;

} // namespace awakening::auto_buff::motion_model
