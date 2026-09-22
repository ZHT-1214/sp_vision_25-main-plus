#pragma once
#include "tasks/auto_aim/type.hpp"
#include "utils/impl.hpp"
namespace awakening::auto_aim {
class ArmorInfer {
public:
    using Ptr = std::unique_ptr<ArmorInfer>;
    ArmorInfer(const YAML::Node& config);
    AWAKENING_IMPL_DEFINITION(ArmorInfer)
    static Ptr create(const YAML::Node& config) {
        return std::make_unique<ArmorInfer>(config);
    }

    [[nodiscard]] std::vector<Armor> process(const cv::Mat& output_buffer) const;

    int input_w() const noexcept;
    int input_h() const noexcept;
    bool use_norm() const noexcept;
    PixelFormat target_format() const noexcept;
};

} // namespace awakening::auto_aim