#pragma once
#include "context.hpp"
#include "type/InputFrame.hpp"
#include "type/InputFrameWithNNResults.hpp"

class Detector : public app::PluginBase
{
public:
    void declare(app::Context &context) override;
};

inline void Detector::declare(app::Context &context)
{
    context.declare_input_buffer<InputFrame>(this);
    context.declare_output_buffer<InputFrameWithNNResults>(this);
}
