#pragma once
#include "utils/common/image.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <vector>
namespace awakening::utils {

class NetDetectorBase {
public:
    struct OutPut {
        std::vector<cv::Mat> outputs;
        Eigen::Matrix3f transform_matrix;
        cv::Mat resized_img;
    };
    struct Config {
        PixelFormat target_format;
        double preprocess_scale;
        int target_w;
        int target_h;
    };
    using Ptr = std::unique_ptr<NetDetectorBase>;

    virtual OutPut detect(const cv::Mat& img, PixelFormat format) = 0;
    virtual ~NetDetectorBase() = default;
};
} // namespace awakening::utils
