#pragma once
#include "type/ECSData.hpp"


struct FireResult
{
    AimMode mode;
    double  yaw = 0.0;
    double  pitch = 0.0;
    bool is_find_target = false;
    bool is_enable_fire = false;
    bool is_find_buff = false;
    bool is_keep_shooting = true;

};
