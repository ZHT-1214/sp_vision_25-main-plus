#pragma once
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/base/common.hpp"
#include "utils/impl.hpp"
#include <yaml-cpp/node/node.h>
namespace awakening::auto_buff {
class RuneAimer {
public:
    RuneAimer(const YAML::Node& node);
    GimbalCmd
    aim(const RuneTarget& target,
        double bullet_speed,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom);
    AWAKENING_IMPL_DEFINITION(RuneAimer)
    std::pair<double, double> get_yaw_pitch_offset();
    void set_operator_offset(std::pair<double, double> offset);
    bool tick_fire(const RuneTarget& _target, const GimbalCmd& cmd) const noexcept;
};
} // namespace awakening::auto_buff