#pragma once

enum class MyColor
{
    Red,
    Blue
};

enum class AimMode
{
    AutoAim = 1,
    SmallRune,
    BigRune,
    OutPost,
    StaticOutPost,
    BaseTop,
    BaseBottom
};

struct ECSData
{
    MyColor my_color;
    AimMode mode;
    bool is_start;   // 比赛开始标志位
    bool is_ready; // 是否可以打弹
    double yaw;   // 云台当前的yaw
    double pitch; // 云台当前的pitch
    double roll;  // 云台当前的roll
};
