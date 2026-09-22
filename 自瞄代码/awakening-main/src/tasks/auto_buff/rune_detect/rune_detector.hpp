#pragma once
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/common.hpp"
#include "utils/impl.hpp"
#include <opencv2/core/types.hpp>
#include <yaml-cpp/node/node.h>
namespace awakening::auto_buff {
class RuneDetector {
public:
    RuneDetector(const YAML::Node& config);
    RuneDetection detect(const CommonFrame& frame, const cv::Rect& focus, EnemyColor enemy_color);
    AWAKENING_IMPL_DEFINITION(RuneDetector)
};
} // namespace awakening::auto_buff