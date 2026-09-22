#pragma once

#include "utils/common/type_common.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <yaml-cpp/node/node.h>
namespace awakening {
namespace auto_aim {
    enum class AutoAimFsm : int {
        AIM_SINGLE_ARMOR,
        AIM_WHOLE_CAR_ARMOR,
        AIM_WHOLE_CAR_PAIR,
        AIM_WHOLE_CAR_CENTER,
    };

    inline std::string string_by_auto_aim_fsm(AutoAimFsm state) {
        constexpr const char* details[] = {
            "AIM_SINGLE_ARMOR",
            "AIM_WHOLE_CAR_ARMOR",
            "AIM_WHOLE_CAR_PAIR",
            "AIM_WHOLE_CAR_CENTER",

        };
        return std::string(details[std::to_underlying(state)]);
    }

    class AutoAimFsmController {
    public:
        struct Params {
            double transfer_time;
            double single_whole_up;
            double single_whole_down;
            double whole_pair_up;
            double whole_pair_down;
            double pair_center_up;
            double pair_center_down;
            void load(const YAML::Node& config) {
                transfer_time = config["transfer_time"]
                    ? config["transfer_time"].as<double>()
                    : config["transfer_thresh"].as<double>() * 0.01;
                single_whole_up = config["single_whole_up"].as<double>();
                single_whole_down = config["single_whole_down"].as<double>();
                whole_pair_up = config["whole_pair_up"].as<double>();
                whole_pair_down = config["whole_pair_down"].as<double>();
                pair_center_up = config["pair_center_up"].as<double>();
                pair_center_down = config["pair_center_down"].as<double>();
            }
        } params_;
        AutoAimFsmController(const YAML::Node& config) {
            params_.load(config);
        }
        AutoAimFsm get_state() const {
            return fsm_state_;
        }
        AutoAimFsm fsm_state_ { AutoAimFsm::AIM_SINGLE_ARMOR };

        double overflow_time_ = 0.0;
        TimePoint last_update_time_ {};
        bool has_last_update_time_ = false;

        void update(double v_yaw, bool target_jumped, const TimePoint& now) {
            const double dt = get_dt(now);
            if (!target_jumped) {
                reset(now);
                return;
            }

            const double av = std::abs(v_yaw);

            switch (fsm_state_) {
                case AutoAimFsm::AIM_SINGLE_ARMOR: {
                    overflow_time_ = (av > params_.single_whole_up) ? overflow_time_ + dt : 0.0;
                    if (overflow_time_ > params_.transfer_time) {
                        transfer_to(AutoAimFsm::AIM_WHOLE_CAR_ARMOR);
                    }
                    break;
                }

                case AutoAimFsm::AIM_WHOLE_CAR_ARMOR: {
                    if (av > params_.whole_pair_up)
                        overflow_time_ += dt;
                    else if (av < params_.single_whole_down)
                        overflow_time_ -= dt;
                    else
                        overflow_time_ = 0.0;

                    if (std::abs(overflow_time_) > params_.transfer_time) {
                        transfer_to(
                            (overflow_time_ > 0.0) ? AutoAimFsm::AIM_WHOLE_CAR_PAIR
                                                   : AutoAimFsm::AIM_SINGLE_ARMOR
                        );
                    }
                    break;
                }

                case AutoAimFsm::AIM_WHOLE_CAR_PAIR: {
                    if (av > params_.pair_center_up)
                        overflow_time_ += dt;
                    else if (av < params_.whole_pair_down)
                        overflow_time_ -= dt;
                    else
                        overflow_time_ = 0.0;

                    if (std::abs(overflow_time_) > params_.transfer_time) {
                        transfer_to(
                            (overflow_time_ > 0.0) ? AutoAimFsm::AIM_WHOLE_CAR_CENTER
                                                   : AutoAimFsm::AIM_WHOLE_CAR_ARMOR
                        );
                    }
                    break;
                }

                case AutoAimFsm::AIM_WHOLE_CAR_CENTER: {
                    overflow_time_ = (av < params_.pair_center_down) ? overflow_time_ + dt : 0.0;
                    if (overflow_time_ > params_.transfer_time) {
                        transfer_to(AutoAimFsm::AIM_WHOLE_CAR_PAIR);
                    }
                    break;
                }

                default:
                    reset(now);
                    break;
            }
        }

    private:
        double get_dt(const TimePoint& now) {
            const double dt = has_last_update_time_
                ? std::chrono::duration<double>(now - last_update_time_).count()
                : 0.0;
            last_update_time_ = now;
            has_last_update_time_ = true;
            return dt > 0.0 ? dt : 0.0;
        }

        void transfer_to(AutoAimFsm state) {
            fsm_state_ = state;
            overflow_time_ = 0.0;
        }

        void reset(const TimePoint& now) {
            fsm_state_ = AutoAimFsm::AIM_SINGLE_ARMOR;
            overflow_time_ = 0.0;
            last_update_time_ = now;
            has_last_update_time_ = true;
        }
    };
} // namespace auto_aim
} // namespace awakening
