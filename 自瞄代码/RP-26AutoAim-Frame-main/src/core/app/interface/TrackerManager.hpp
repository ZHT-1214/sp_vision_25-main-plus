#pragma once
#include "context.hpp"
#include "type/InputFrameWithNNResults.hpp"
#include "type/TrackResult.hpp"

class TrackerManager : public app::PluginBase
{
public:
    void declare(app::Context &context) override;
};

inline void TrackerManager::declare(app::Context &context)
{
    context.declare_input_buffer<InputFrameWithNNResults>(this);
    context.declare_output_buffer<TrackResult>(this);
}
