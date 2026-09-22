#pragma once
#include "opencv2/core/mat.hpp"
#include "utils/common/type_common.hpp"
namespace awakening::vslam {
class Frame{
public:

    ISO3 pose;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat img_gray;
    TimePoint timestamp;
    int id;
    bool detected = false;
};
}