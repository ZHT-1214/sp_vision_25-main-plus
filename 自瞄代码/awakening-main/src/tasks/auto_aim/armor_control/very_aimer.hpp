#pragma once
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/base/common.hpp"
#include "utils/impl.hpp"
#include <memory>
#include <yaml-cpp/node/node.h>

namespace awakening::auto_aim {
class VeryAimer {
public:
    VeryAimer(const YAML::Node& config);
    AWAKENING_IMPL_DEFINITION(VeryAimer)
    [[nodiscard]] GimbalCmd very_aim(
        const ArmorTarget& target,
        double bullet_speed,
        const AutoAimFsm& fsm,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom
    );
    std::pair<double, double> get_yaw_pitch_offset();
    void set_operator_offset(std::pair<double, double> offset);
};
} // namespace awakening::auto_aim