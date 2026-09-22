#pragma once
#include "tasks/auto_aim/type.hpp"
#include "utils/impl.hpp"
#include <optional>
#include <vector>
namespace awakening::auto_aim {
class ArmorDetector {
public:
    using Ptr = std::unique_ptr<ArmorDetector>;
    ArmorDetector(const YAML::Node& config);
    [[nodiscard]] std::tuple<std::vector<Light>, std::vector<Armor>> detect(
        const CommonFrame& frame,
        const cv::Rect& net_focus,
        const std::optional<cv::Rect>& detect_light = std::nullopt
    );
    double get_net_wh_ratio() const noexcept;
    AWAKENING_IMPL_DEFINITION(ArmorDetector)
};
} // namespace awakening::auto_aim