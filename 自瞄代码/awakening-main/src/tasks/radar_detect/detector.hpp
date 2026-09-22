#pragma once
#include "tasks/base/common.hpp"
#include "tasks/radar_detect/type.hpp"
#include "type.hpp"
#include "utils/impl.hpp"
#include <opencv2/core/types.hpp>
#include <vector>
#include <yaml-cpp/node/node.h>
namespace awakening::radar_detect {
class Detector {
public:
    Detector(const YAML::Node& config);
    std::vector<Car> detect(const CommonFrame& frame, const cv::Rect& focus);
    std::vector<Armor> detect_armors(const CommonFrame& frame, const cv::Rect& focus);
    AWAKENING_IMPL_DEFINITION(Detector)
};
} // namespace awakening::radar_detect