#pragma once 
#include <opencv2/core.hpp>

#include "type/ECSData.hpp"
#include "time/time.hpp"

struct InputFrame
{
    ECSData ecs_data;
    timetool::Timestamp timestamp;
    cv::Mat img;
};
