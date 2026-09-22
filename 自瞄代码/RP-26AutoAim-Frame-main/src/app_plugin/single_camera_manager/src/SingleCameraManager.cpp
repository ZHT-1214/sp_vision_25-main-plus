#include "SingleCameraManager.hpp"

#include "class_loader.hpp"
#include "time/time.hpp"

#include <chrono>
#include <cstdint>
#include <glog/logging.h>
#include <thread>

namespace app_plugin
{

void SingleCameraManager::process(const app::Context &context)
{
    static std::uint64_t cycles = 0;
    ++cycles;
    if (cycles == 1 || cycles % 100 == 0)
        LOG(INFO) << "[CameraManager] running, cycles=" << cycles;

    static auto input = context.get_channel_subscriber<ECSData>(this);
    static auto output = context.get_buffer_publisher<InputFrame>(this);

    const auto ecs_data = input.wait_next();
    InputFrame frame;
    frame.ecs_data = *ecs_data;
    frame.timestamp = timetool::now();
    output.push(std::move(frame));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace app_plugin

REGISTER_PLUGIN("CameraManager", app_plugin::SingleCameraManager)
