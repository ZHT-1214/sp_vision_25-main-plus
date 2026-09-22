#include "MinimalTrackerManager.hpp"

#include "class_loader.hpp"

#include <cstdint>
#include <glog/logging.h>

void app_plugin::MinimalTrackerManager::process(const app::Context &context)
{
    static std::uint64_t cycles = 0;
    ++cycles;
    if (cycles == 1 || cycles % 100 == 0)
        LOG(INFO) << "[TrackerManager] running, cycles=" << cycles;

    static auto input = context.get_buffer_subscriber<InputFrameWithNNResults>(this);
    static auto output = context.get_buffer_publisher<TrackResult>(this);

    const auto frame = input.wait_pop();
    TrackResult result;
    result.timestamp = frame.input_frame.timestamp;
    result.mode = frame.input_frame.ecs_data.mode;
    output.push(std::move(result));
}

REGISTER_PLUGIN("TrackerManager", app_plugin::MinimalTrackerManager)
