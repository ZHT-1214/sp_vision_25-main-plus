#pragma once
#include <vector>
#include <opencv2/core.hpp>

#include "type/InputFrame.hpp"


struct NNRuneInfo
{
    cv::Point top, left, right, bottom , point_R;
    int class_id;//0是未击打，1是已击打
};

struct NNArmorInfo
{
    enum class NNColor
    {
        Blue,
        Red,
        White
    };
    std::vector<cv::Point2d> end_points;
    int size;
    int num;
    NNColor nn_color;
};

struct InputFrameWithNNResults
{
    InputFrame input_frame;
    std::vector<NNArmorInfo> nn_armor_infos;
    std::vector<NNRuneInfo> nn_rune_infos;
};
