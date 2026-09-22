#pragma once

#include "ReceiveDecoder.hpp"

namespace app_plugin
{

// Emits a neutral ECS frame instead of decoding hardware.
class MinimalReceiveDecoder final : public ReceiveDecoder
{
public:
    void process(const app::Context &context) override;
};

} // namespace app_plugin
