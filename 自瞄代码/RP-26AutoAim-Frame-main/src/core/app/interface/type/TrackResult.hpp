#pragma once
#include <opencv2/core.hpp>

#include "type/ECSData.hpp"
#include "transform_tools/transform_tools.h"
#include "time/time.hpp"

enum KalmanState
{
    X_Center = 0, // 转轴中心X坐标
    V_X_Center,   // 转轴中心X方向平动速度
    Y_Armor,      // 装甲板中心的Y坐标
    V_Y_Armor,    // 装甲板中心的Y的速度
    Z_Center,     // 转轴中心Z坐标
    V_Z_Center,   // 转轴中心Z方向速度
    Theta,        // 和Z轴的夹角，俯视下，顺时针方向为正(右手系)
    V_W,          // 角速度
    Radius        // 当前转轴的半径
};

struct Aim
{
    Eigen::Vector<double, 9> armor_state;        // 卡尔曼状态量
    int armor_class;                             // 装甲板的类别
    double another_R;                            // 另一个半径
    double delta_y;                              // 另一个y与这个当前这个装甲板y的补偿
};

struct OutPost
{
    enum Height 
    {
        Low,
        Mid,
        High
    };

    Eigen::Vector<double, 9> armor_state;        // 当前装甲板卡尔曼状态量
    Height height;                               // 当前装甲板处于什么位置
    bool is_tracking = false;                    // 前哨建模标志位
};

struct TrackResult
{
    cv::Mat img;
    timetool::Timestamp timestamp;
    transform_tools::TFTree tf_tree;
    AimMode mode;
    std::vector<Aim> aims;//一般自瞄
    OutPost outpost;//前哨
};
