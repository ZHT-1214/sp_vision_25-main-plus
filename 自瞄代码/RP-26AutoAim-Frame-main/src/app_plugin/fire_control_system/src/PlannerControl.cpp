#include "PlannerControl.hpp"

#include "class_loader.hpp"

#include <cstdint>
#include <glog/logging.h>

void PlannerControl::process(const app::Context &context)
{
    static std::uint64_t cycles = 0;
    ++cycles;
    if (cycles == 1 || cycles % 100 == 0)
        LOG(INFO) << "[PlannerControl] running, cycles=" << cycles;

    static auto ecs_input = context.get_channel_subscriber<ECSData>(this);
    static auto track_input = context.get_buffer_subscriber<TrackResult>(this);
    static auto output = context.get_buffer_publisher<FireResult>(this);

    const auto ecs_data = ecs_input.wait_next();
    (void)track_input.wait_pop();

    FireResult result;
    result.mode = ecs_data->mode;
    output.push(std::move(result));
}

REGISTER_PLUGIN("PlannerControl", PlannerControl)
