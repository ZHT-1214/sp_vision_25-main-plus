#include "very_aimer.hpp"
#include "angles.h"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/dta_utils.hpp"
#include "utils/common/type_common.hpp"
#include "utils/logger.hpp"
#include "utils/utils.hpp"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace awakening::auto_aim {
struct VeryAimer::Impl {
    struct Params {
        double sample_total_time;
        int sample_horizon;
        double fire_delay_min;
        double fire_delay_max;
        double max_yaw_acc;
        double max_pitch_acc;
        double prediction_delay;
        double aim_center_more_prediction_time;
        double shooting_range_h;
        double shooting_range_w_small;
        double shooting_range_w_large;
        double min_enable_pitch_deg;
        double min_enable_yaw_deg;

        void load(const YAML::Node& config) {
            sample_total_time = config["sample_total_time"].as<double>();
            sample_horizon = config["sample_horizon"].as<int>();
            fire_delay_min = config["fire_delay_min"].as<double>();
            fire_delay_max = config["fire_delay_max"].as<double>();
            max_yaw_acc = config["max_yaw_acc"].as<double>();
            max_pitch_acc = config["max_pitch_acc"].as<double>();
            prediction_delay = config["prediction_delay"].as<double>();
            aim_center_more_prediction_time =
                config["aim_center_more_prediction_time"].as<double>();
            shooting_range_h = config["shooting_range_h"].as<double>();
            shooting_range_w_small = config["shooting_range_w_small"].as<double>();
            shooting_range_w_large = config["shooting_range_w_large"].as<double>();
            min_enable_pitch_deg = config["min_enable_pitch_deg"].as<double>();
            min_enable_yaw_deg = config["min_enable_yaw_deg"].as<double>();
        }
    } params_;
    using MPCType = dta_utils::TinyMpcTrajectory;
    Impl(const YAML::Node& config) {
        params_.load(config);
        ballistic_trajectory_ = BallisticTrajectory::create(config["ballistic_trajectory"]);
        base_yaw_offset_rad_ = angles::from_degrees(config["base_yaw_offset"].as<double>());
        base_pitch_offset_rad_ = angles::from_degrees(config["base_pitch_offset"].as<double>());
        auto type = config["type"].as<std::string>();
        if (type == "mpc" || type == "MPC") {
            MPCType::Params mpc_p {
                .yaw_max_acc = (float)params_.max_yaw_acc,
                .pitch_max_acc = (float)params_.max_pitch_acc,
                .horizon = params_.sample_horizon,
                .dt = params_.sample_total_time / params_.sample_horizon,
                .max_iter = 10,
            };
            mpc_traj_ = MPCType::create(mpc_p);
            AWAKENING_INFO("very_aimer: using  MpcTrajectory");
        }
    }
    [[nodiscard]] int
    select_armor(const ArmorTarget& target, const AutoAimFsm& auto_aim_fsm) const noexcept {
        static int lock_id = -1;
        const auto& target_state = target.get_target_state();
        const auto armors_pose = target_state.get_armors_pose(target.target_number);
        const int armor_num = static_cast<int>(armors_pose.size());
        int i_chosen = 0;
        const double center_yaw = std::atan2(target_state.pos().y(), target_state.pos().x());
        auto delta_angle = [&](int i) {
            auto rpy = utils::matrix2rpy<double>(armors_pose[i].linear());
            // return angles::normalize_angle(
            //     rpy[2]
            //     - std::atan2(armors_pose[i].translation().y(), armors_pose[i].translation().x())
            // );
            return angles::normalize_angle(rpy[2] - center_yaw);
        };
        auto abs_delta_angle = [&](int i) { return std::abs(delta_angle(i)); };
        auto pick_best_range = [&](int first, int last, auto&& accept) -> int {
            int best = -1;
            double best_val = std::numeric_limits<double>::infinity();
            for (int i = first; i < last; ++i) {
                if (!accept(i))
                    continue;
                const double val = abs_delta_angle(i);
                if (val < best_val) {
                    best_val = val;
                    best = i;
                }
            }
            return best;
        };
        auto accept_all = [](int) { return true; };
        auto accept_pair = [&](int i) { return (i == 0 || i == 2); };

        if (auto_aim_fsm == AutoAimFsm::AIM_SINGLE_ARMOR
            && target.target_number != ArmorClass::OUTPOST && armor_num > 0)
        {
            int candidates[2] = { -1, -1 };
            int candidate_count = 0;
            for (int i = 0; i < armor_num && candidate_count < 2; ++i) {
                if (std::abs(delta_angle(i)) <= 60.0 / 57.3) {
                    candidates[candidate_count++] = i;
                }
            }

            if (candidate_count > 0) {
                int pick = -1;

                if (candidate_count == 1) {
                    pick = candidates[0];
                    lock_id = -1;
                } else {
                    if (lock_id < 0 || (lock_id != candidates[0] && lock_id != candidates[1])) {
                        lock_id = (abs_delta_angle(candidates[0]) < abs_delta_angle(candidates[1]))
                            ? candidates[0]
                            : candidates[1];
                    }
                    pick = lock_id;
                }

                if (pick >= 0 && pick < armor_num) {
                    i_chosen = pick;
                }
            }

            return i_chosen;
        }
        if (armor_num > 0) {
            int best_idx = -1;

            if (auto_aim_fsm
                    == AutoAimFsm::AIM_WHOLE_CAR_PAIR //4选2,本质提升控制轨迹与目标轨迹重合窗口
                && target.target_number != ArmorClass::OUTPOST)
            {
                best_idx = pick_best_range(0, armor_num, accept_pair);
            }
            if (best_idx < 0) {
                best_idx = pick_best_range(0, armor_num, accept_all);
            }

            i_chosen = best_idx;
        }

        return i_chosen;
    }
    [[nodiscard]] dta_utils::ControlPoint get_control_point(
        const ISO3& armor_pose,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom,
        double bullet_speed,
        int aim_id
    ) const noexcept {
        dta_utils::ControlPoint cp;
        auto p = armor_pose.translation() - shoot_in_gimbal_odom.translation();
        auto desired_pitch_opt = ballistic_trajectory_->solve_pitch(p, bullet_speed);
        if (!desired_pitch_opt) {
            cp.valid = false;
            AWAKENING_ERROR(
                "very_aimer: get_control_point: Failed to solve pitch armor_pos: [{}, {}, {}], bullet_speed: {}",
                p.x(),
                p.y(),
                p.z(),
                bullet_speed
            );
            return cp;
        }
        const auto [yaw_offset, pitch_offset] = get_yaw_pitch_offset();
        const double desired_control_yaw = std::atan2(p.y(), p.x());
        auto desired_shoot = utils::rpy2matrix(Vec3(
            0.0,
            desired_pitch_opt.value() + pitch_offset,
            angles::normalize_angle(desired_control_yaw + yaw_offset)
        ));
        const Mat3 R_gimbal_shoot =
            gimbal_in_gimbal_odom.linear().transpose() * shoot_in_gimbal_odom.linear();
        auto desired_gimbal = desired_shoot * R_gimbal_shoot.transpose();
        auto rpy = utils::matrix2rpy<double>(desired_gimbal);
        cp.yaw = rpy[2];
        cp.pitch = rpy[1];
        cp.valid = true;

        cp.aim_point.pose = armor_pose;
        cp.aim_id = aim_id;
        return cp;
    };
    [[nodiscard]] dta_utils::ControlPoint select_and_get_control_point(
        const ArmorTarget& target,
        double bullet_speed,
        const AutoAimFsm& fsm,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom
    ) const noexcept {
        const auto& target_state = target.get_target_state();
        const int selected_armor = select_armor(target, fsm);
        auto armors_pose = target_state.get_armors_pose(target.target_number);
        if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) { //瞄准中间把目标推回原来板子的位置
            auto p = target_state.pos() - shoot_in_gimbal_odom.translation();
            double center_xy_dis = std::hypot(p.x(), p.y());
            double center_yaw = std::atan2(p.y(), p.x());
            center_xy_dis -= target_state.get_armor_r(selected_armor, target.target_number);
            armors_pose[selected_armor].translation().x() = center_xy_dis * std::cos(center_yaw);
            armors_pose[selected_armor].translation().y() = center_xy_dis * std::sin(center_yaw);
        }
        return get_control_point(
            armors_pose[selected_armor],
            shoot_in_gimbal_odom,
            gimbal_in_gimbal_odom,
            bullet_speed,
            selected_armor
        );
    }
    struct HitCtx {
        ArmorTarget hit_time_target;
        double fly_time;
    };
    std::optional<HitCtx> get_hit(
        const ArmorTarget& target_ready_to_aim,
        double bullet_speed,
        const AutoAimFsm& fsm,
        const ISO3& shoot_in_gimbal_odom
    ) const noexcept {
        auto hit_time_target = target_ready_to_aim.fast_copy_without_ekf();
        const int roughly_select = select_armor(hit_time_target, fsm);
        const auto __armors_pose =
            hit_time_target.get_target_state().get_armors_pose(hit_time_target.target_number);
        auto prev_pitch_and_fly_time_opt = ballistic_trajectory_->solve_pitch_and_flytime(
            __armors_pose[roughly_select].translation() - shoot_in_gimbal_odom.translation(),
            bullet_speed
        );
        if (!prev_pitch_and_fly_time_opt) {
            return std::nullopt;
        }
        auto prev_fly_time = prev_pitch_and_fly_time_opt.value().second;

        for (int iter = 0; iter < 10; ++iter) {
            auto i_target = hit_time_target.fast_copy_without_ekf();
            i_target.set_target_state([&](armor_point_motion_model::State& state) {
                state.predict(prev_fly_time, i_target.target_number);
            });
            // auto iter_select = select_armor(i_target, fsm);//不知道哪个最好
            const auto iter_armors_pose =
                i_target.get_target_state().get_armors_pose(i_target.target_number);
            // auto iter_pitch_and_fly_time_opt = ballistic_trajectory_->solve_pitch_and_flytime(
            //     iter_armors_xyza[iter_select].head<3>(),
            //     bullet_speed
            // );
            auto iter_pitch_and_fly_time_opt = ballistic_trajectory_->solve_pitch_and_flytime(
                iter_armors_pose[roughly_select].translation() - shoot_in_gimbal_odom.translation(),
                bullet_speed
            );
            if (!iter_pitch_and_fly_time_opt) {
                return std::nullopt;
            }
            if (std::abs(iter_pitch_and_fly_time_opt.value().second - prev_fly_time) < 1e-3) {
                prev_fly_time = iter_pitch_and_fly_time_opt.value().second;
                break;
            }
            prev_fly_time = iter_pitch_and_fly_time_opt.value().second;
        }
        const double predict_time = prev_fly_time + params_.prediction_delay
            + (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER ? params_.aim_center_more_prediction_time : 0
            ); //瞄准中间只能预测一段发弹延迟
        hit_time_target.set_target_state([&](auto& state) {
            state.predict(predict_time, hit_time_target.target_number);
        });
        return HitCtx {
            .hit_time_target = hit_time_target,
            .fly_time = prev_fly_time,
        };
    }
    struct InputCtx {
        ArmorTarget target;
        double bullet_speed;
        AutoAimFsm fsm;
        ISO3 shoot_in_gimbal_odom;
        ISO3 gimbal_in_gimbal_odom;
        bool valid = false;
        bool is_same(
            const ArmorTarget& _target,
            double _bullet_speed,
            AutoAimFsm _fsm,
            const ISO3& _shoot_in_gimbal_odom,
            const ISO3& _gimbal_in_gimbal_odom
        ) {
            if (!valid)
                return false;
            bool same_target = _target.this_id == target.this_id;
            return same_target && _fsm == fsm && std::abs(_bullet_speed - bullet_speed) < 1e-2
                && _shoot_in_gimbal_odom.isApprox(shoot_in_gimbal_odom, 1e-2)
                && _gimbal_in_gimbal_odom.isApprox(gimbal_in_gimbal_odom, 1e-2);
        }
        void
        set(const ArmorTarget& _target,
            double _bullet_speed,
            AutoAimFsm _fsm,
            const ISO3& _shoot_in_gimbal_odom,
            const ISO3& _gimbal_in_gimbal_odom) {
            target = _target.fast_copy_without_ekf();
            bullet_speed = _bullet_speed;
            fsm = _fsm;
            shoot_in_gimbal_odom = _shoot_in_gimbal_odom;
            gimbal_in_gimbal_odom = _gimbal_in_gimbal_odom;
            valid = true;
        }

    } last_input_;
    TimePoint last_time_;
    dta_utils::LimitTrajectory limit_traj_;
    dta_utils::ControlPoint limit_traj_cp0_;
    Trajectory<AimPoint, double> aim_traj_;
    Trajectory<AimPoint, double> aim_center_aim_traj_;
    Trajectory<dta_utils::GimbalState, double> aim_center_target_traj_;
    dta_utils::ControlPoint aim_center_target_traj_cp0_;
    double last_fly_time_;
    int last_select_;
    [[nodiscard]] GimbalCmd very_aim(
        const ArmorTarget& _target,
        double bullet_speed,
        AutoAimFsm fsm,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom
    ) noexcept {
        if (_target.target_number == ArmorClass::BASE) {
            fsm = AutoAimFsm::AIM_SINGLE_ARMOR;
        }
        GimbalCmd cmd;
        cmd.appear = false;
        bool is_same =
            last_input_
                .is_same(_target, bullet_speed, fsm, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
        double time_in_traj = 0.0;
        const auto now = Clock::now();
        if (is_same) { //状态完全未更新，使用第一次构建的轨迹基于时间进行推进减少重复计算
            time_in_traj = std::chrono::duration<double>(now - last_time_).count();
        } else {
            last_input_
                .set(_target, bullet_speed, fsm, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
            last_time_ = now;
        }
        auto target = _target.fast_copy_without_ekf();
        target.set_target_state([&](armor_point_motion_model::State& state) {
            state.predict(now, target.target_number);
        });
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int horizon = make_even(params_.sample_horizon);
        const double dt = params_.sample_total_time / horizon;
        auto sample_once = [&](double t,
                               const ArmorTarget& base_target,
                               AutoAimFsm fsm_mode,
                               const dta_utils::ControlPoint& _cp0,
                               dta_utils::GimbalState& out_gs,
                               AimPoint& out_ap) -> bool {
            auto tmp_target = base_target.fast_copy_without_ekf();
            tmp_target.set_target_state([&](armor_point_motion_model::State& state) {
                state.predict(t, tmp_target.target_number);
            }); //保证轨迹完全为不同时间对同一状态的瞄准控制点
            auto hit_opt = get_hit(tmp_target, bullet_speed, fsm_mode, shoot_in_gimbal_odom);
            if (!hit_opt)
                return false;
            auto cp = select_and_get_control_point(
                hit_opt->hit_time_target,
                bullet_speed,
                fsm_mode,
                shoot_in_gimbal_odom,
                gimbal_in_gimbal_odom
            );
            if (!cp.valid)
                return false;
            out_gs.aim_id = cp.aim_id;
            out_gs.yaw_state.p = angles::normalize_angle(cp.yaw - _cp0.yaw);
            out_gs.pitch_state.p = angles::normalize_angle(cp.pitch - _cp0.pitch);
            out_ap = cp.aim_point;
            return true;
        };
        auto push_sample = [&](auto& traj_gs,
                               auto& traj_ap,
                               double t,
                               const dta_utils::ControlPoint& _cp0,
                               AutoAimFsm fsm_mode) -> bool {
            dta_utils::GimbalState gs;
            AimPoint ap;
            if (!sample_once(t, target, fsm_mode, _cp0, gs, ap)) {
                return false;
            }
            traj_gs.push_back(gs, t);
            traj_ap.push_back(ap, t);
            return true;
        };
        if (!is_same) {
            auto hit_ctx_opt =
                get_hit(target, bullet_speed, fsm, shoot_in_gimbal_odom); //直接算到击中目标时间
            if (!hit_ctx_opt) {
                return cmd;
            }
            const auto& hit_ctx = *hit_ctx_opt;
            auto cp0 = select_and_get_control_point(
                hit_ctx.hit_time_target,
                bullet_speed,
                fsm,
                shoot_in_gimbal_odom,
                gimbal_in_gimbal_odom
            );
            if (!cp0.valid) {
                return cmd;
            }
            last_fly_time_ = hit_ctx.fly_time;
            last_select_ = cp0.aim_id;
            auto build_traj = [&](auto& traj_gs,
                                  auto& traj_ap,
                                  const dta_utils::ControlPoint& _cp0,
                                  AutoAimFsm fsm_mode,
                                  int horizon,
                                  double dt) -> bool {
                int half = horizon / 2;
                for (int i = half; i >= 1; --i) {
                    if (!push_sample(traj_gs, traj_ap, -i * dt, _cp0, fsm_mode)) {
                        return false;
                    }
                }

                dta_utils::GimbalState gs0;
                gs0.aim_id = _cp0.aim_id;
                gs0.yaw_state.p = 0.0;
                gs0.pitch_state.p = 0.0;
                traj_gs.push_back(gs0, 0.0);
                traj_ap.push_back(_cp0.aim_point, 0.0);

                for (int i = 1; i <= half; ++i) {
                    if (!push_sample(traj_gs, traj_ap, i * dt, _cp0, fsm_mode)) {
                        return false;
                    }
                }

                return true;
            };
            limit_traj_cp0_ = cp0;
            limit_traj_.clear();
            aim_traj_.clear();
            limit_traj_.reserve(horizon + 1);
            aim_traj_.reserve(horizon + 1);

            if (!build_traj(limit_traj_, aim_traj_, limit_traj_cp0_, fsm, horizon, dt)) {
                return cmd;
            }
            if (!mpc_traj_) {
                limit_traj_.build_limit(params_.max_yaw_acc, params_.max_pitch_acc, time_in_traj);
            } else {
                if (!mpc_traj_->solve(limit_traj_, time_in_traj)) {
                    AWAKENING_ERROR("very_aimer: mpc solve failed");
                    return cmd;
                }
            }

            if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) { //瞄准中间的目标和控制不一样
                aim_center_target_traj_.clear();
                aim_center_target_traj_.reserve(horizon + 1);
                auto aim_center_target_hit_ctx_opt = get_hit(
                    target,
                    bullet_speed,
                    AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                    shoot_in_gimbal_odom
                );
                if (!aim_center_target_hit_ctx_opt) {
                    return cmd;
                }
                const auto& aim_center_target_hit_ctx = *aim_center_target_hit_ctx_opt;
                aim_center_target_traj_cp0_ = select_and_get_control_point(
                    aim_center_target_hit_ctx.hit_time_target,
                    bullet_speed,
                    AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                    shoot_in_gimbal_odom,
                    gimbal_in_gimbal_odom
                );
                aim_center_aim_traj_.clear();
                aim_center_aim_traj_.reserve(horizon + 1);
                if (!build_traj(
                        aim_center_target_traj_,
                        aim_center_aim_traj_,
                        aim_center_target_traj_cp0_,
                        AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                        horizon,
                        dt
                    ))
                {
                    return cmd;
                }
            }
        } else {
            const int target_forward_idx = ((int)(horizon / 2) + time_in_traj / dt);
            auto replenish_traj = [&](auto& traj_gs,
                                      auto& traj_ap,
                                      const dta_utils::ControlPoint& _cp0,
                                      AutoAimFsm fsm_mode,
                                      int target_forward_idx,
                                      double dt) -> bool {
                if (traj_gs.empty())
                    return false;
                int last_forward_idx =
                    static_cast<int>(std::floor(traj_gs.time_at(traj_gs.size() - 1) / dt));
                for (int i = last_forward_idx + 1; i <= target_forward_idx; ++i) {
                    if (!push_sample(traj_gs, traj_ap, i * dt, _cp0, fsm_mode)) {
                        return false;
                    }
                }
                return true;
            };
            const size_t old_limit_size = limit_traj_.size();
            if (!replenish_traj(
                    limit_traj_,
                    aim_traj_,
                    limit_traj_cp0_,
                    fsm,
                    target_forward_idx,
                    dt
                )) {
                return cmd;
            }
            if (limit_traj_.size() > old_limit_size) {
                if (!mpc_traj_) {
                    limit_traj_.update_limit_after_append(
                        params_.max_yaw_acc,
                        params_.max_pitch_acc,
                        time_in_traj,
                        old_limit_size
                    );
                } else {
                    if (!mpc_traj_->solve(limit_traj_, time_in_traj)) {
                        AWAKENING_ERROR("very_aimer: mpc solve failed");
                        return cmd;
                    }
                }
            }
            if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) { //瞄准中间的目标和控制不一样
                if (!replenish_traj(
                        aim_center_target_traj_,
                        aim_center_aim_traj_,
                        aim_center_target_traj_cp0_,
                        AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                        target_forward_idx,
                        dt
                    ))
                {
                    return cmd;
                }
            }
        }

        const bool use_center_target = fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER;
        const dta_utils::ControlPoint& target_traj_cp0 =
            use_center_target ? aim_center_target_traj_cp0_ : limit_traj_cp0_;
        const Trajectory<dta_utils::GimbalState, double>& target_traj = use_center_target
            ? static_cast<const Trajectory<dta_utils::GimbalState, double>&>(aim_center_target_traj_
            )
            : static_cast<const Trajectory<dta_utils::GimbalState, double>&>(limit_traj_);
        const Trajectory<AimPoint, double>& aim_traj =
            use_center_target ? aim_center_aim_traj_ : aim_traj_;
        auto target_gimbal_state = target_traj.Trajectory::state_at(time_in_traj);
        //目标轨迹，一定击中目标
        auto get_control = [&](double t) {
            if (!mpc_traj_) {
                return limit_traj_.dta_utils::LimitTrajectory::state_at(t);
            } else {
                return mpc_traj_->state_at(t);
            }
        };
        auto control = get_control(time_in_traj);
        //控制轨迹，轨迹优化后最优控制（并非最优，下位机实际vel acc 可以基于error和上位机规划叠加）
        double control_yaw = angles::normalize_angle(control.yaw_state.p + limit_traj_cp0_.yaw);
        double control_pitch =
            angles::normalize_angle(control.pitch_state.p + limit_traj_cp0_.pitch);
        double target_yaw =
            angles::normalize_angle(target_gimbal_state.yaw_state.p + target_traj_cp0.yaw);
        double target_pitch =
            angles::normalize_angle(target_gimbal_state.pitch_state.p + target_traj_cp0.pitch);
        cmd.timestamp = Clock::now();
        cmd.yaw = angles::to_degrees(control_yaw);
        cmd.v_yaw = angles::to_degrees(control.yaw_state.v);
        cmd.a_yaw = angles::to_degrees(control.yaw_state.a);
        cmd.pitch = angles::to_degrees(control_pitch);
        cmd.v_pitch = angles::to_degrees(control.pitch_state.v);
        cmd.a_pitch = angles::to_degrees(control.pitch_state.a);
        cmd.target_yaw = angles::to_degrees(target_yaw);
        cmd.target_pitch = angles::to_degrees(target_pitch);
        cmd.fly_time = last_fly_time_;
        cmd.appear = true;
        cmd.aim_point = aim_traj.state_at(time_in_traj);
        cmd.aim_point.frame_id = target.get_target_state().frame_id;
        cmd.select_id = last_select_;
        const bool is_big = target.target_number == ArmorClass::NO1;
        const double half_w =
            (is_big ? params_.shooting_range_w_large : params_.shooting_range_w_small) * 0.5;
        const double half_h = params_.shooting_range_h * 0.5;
        const double min_enable_yaw = angles::from_degrees(params_.min_enable_yaw_deg);
        const double min_enable_pitch = angles::from_degrees(params_.min_enable_pitch_deg);
        const double pitch_factor = std::cos(FIFTTEN_DEGREE_RAD);
        auto cal_enbale_diff = [&](double _t) {
            auto aim_point = aim_traj.state_at(_t);
            const auto& pos = aim_point.pose.translation();
            const double distance = pos.norm();
            auto pose1_in_aim_point = ISO3::Identity();
            pose1_in_aim_point.translation() = Vec3(0, -half_w, 0);
            auto pose2_in_aim_point = ISO3::Identity();
            pose2_in_aim_point.translation() = Vec3(0, half_w, 0);
            auto pose1_in_odom = aim_point.pose * pose1_in_aim_point;
            auto pose2_in_odom = aim_point.pose * pose2_in_aim_point;
            auto yaw1 =
                std::atan2(pose1_in_odom.translation().y(), pose1_in_odom.translation().x());
            auto yaw2 =
                std::atan2(pose2_in_odom.translation().y(), pose2_in_odom.translation().x());
            auto aim_yaw = std::atan2(pos.y(), pos.x());
            double shooting_range_yaw = std::min(
                std::abs(angles::normalize_angle(yaw1 - aim_yaw)),
                std::abs(angles::normalize_angle(yaw2 - aim_yaw))
            ); //直接算两个边缘yaw
            double shooting_range_pitch = std::abs(std::atan2(half_h, distance));
            auto rpy = utils::matrix2rpy<double>(aim_point.pose.linear(), utils::RPYOrder::XYZ);
            shooting_range_pitch *= std::cos(rpy[1]);
            shooting_range_yaw = std::max(shooting_range_yaw, min_enable_yaw);
            shooting_range_pitch = std::max(shooting_range_pitch, min_enable_pitch);
            return std::make_pair(std::abs(shooting_range_yaw), std::abs(shooting_range_pitch));
        };
        auto enable_diff = cal_enbale_diff(time_in_traj);
        cmd.enable_yaw_diff = angles::to_degrees(enable_diff.first);
        cmd.enable_pitch_diff = angles::to_degrees(enable_diff.second);
        auto abs_angle_error = [](double from, double to) {
            return std::abs(angles::shortest_angular_distance(from, to));
        };
        cmd.fire_advice =
            abs_angle_error(angles::from_degrees(cmd.target_yaw), angles::from_degrees(cmd.yaw))
                < cmd.enable_yaw_diff
            && abs_angle_error(
                   angles::from_degrees(cmd.target_pitch),
                   angles::from_degrees(cmd.pitch)
               ) < cmd.enable_pitch_diff;
        if (fsm != AutoAimFsm::AIM_WHOLE_CAR_CENTER) {
            auto delay_fire = [&](double delay) {
                auto delay_control = get_control(time_in_traj + delay);
                auto delay_target = target_traj.Trajectory::state_at(time_in_traj + delay);
                auto delay_enable = cal_enbale_diff(time_in_traj + delay);
                const double control_yaw =
                    angles::normalize_angle(delay_control.yaw_state.p + limit_traj_cp0_.yaw);
                const double target_yaw =
                    angles::normalize_angle(delay_target.yaw_state.p + target_traj_cp0.yaw);
                const double control_pitch =
                    angles::normalize_angle(delay_control.pitch_state.p + limit_traj_cp0_.pitch);
                const double target_pitch =
                    angles::normalize_angle(delay_target.pitch_state.p + target_traj_cp0.pitch);

                return abs_angle_error(control_yaw, target_yaw) < delay_enable.first
                    && abs_angle_error(control_pitch, target_pitch) < delay_enable.second;
            };
            {
                bool has_can = false;

                double t_check = 0 + params_.fire_delay_min; //发射延迟内不让打弹
                while (t_check < (0 + params_.fire_delay_max) && t_check <= horizon / 2.0) {
                    if (!delay_fire(+t_check)) {
                        cmd.no_shoot();
                    } else {
                        has_can = true;
                    }
                    t_check += (dt / 2.0);
                }
                // if (!has_can) {
                //     cmd.no_shoot();
                // }
            }
        }

        return cmd;
    }
    void set_operator_offset(std::pair<double, double> offset) {
        operator_offset_ = offset;
    }
    std::pair<double, double> get_yaw_pitch_offset() const noexcept {
        return std::make_pair(
            base_yaw_offset_rad_ + operator_offset_.first, //操作手在线调偏置
            base_pitch_offset_rad_ + operator_offset_.second
        );
    }
    BallisticTrajectory::Ptr ballistic_trajectory_;
    double base_yaw_offset_rad_;
    double base_pitch_offset_rad_;
    std::pair<double, double> operator_offset_ = std::make_pair(0, 0);
    MPCType::Ptr mpc_traj_;
};
VeryAimer::VeryAimer(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
VeryAimer::~VeryAimer() noexcept {
    _impl.reset();
}
GimbalCmd VeryAimer::very_aim(
    const ArmorTarget& target,
    double bullet_speed,
    const AutoAimFsm& fsm,
    const ISO3& shoot_in_gimbal_odom,
    const ISO3& gimbal_in_gimbal_odom
) {
    return _impl->very_aim(target, bullet_speed, fsm, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
}
std::pair<double, double> VeryAimer::get_yaw_pitch_offset() {
    return _impl->get_yaw_pitch_offset();
}
void VeryAimer::set_operator_offset(std::pair<double, double> offset) {
    _impl->set_operator_offset(offset);
}
} // namespace awakening::auto_aim
