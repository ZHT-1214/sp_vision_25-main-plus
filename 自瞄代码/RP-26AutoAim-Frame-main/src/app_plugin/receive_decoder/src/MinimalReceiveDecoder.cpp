#include "MinimalReceiveDecoder.hpp"

#include "class_loader.hpp"

#include <cstdint>
#include <chrono>
#include <glog/logging.h>
#include <memory>
#include <thread>

namespace app_plugin
{

void MinimalReceiveDecoder::process(const app::Context &context)
{
    static std::uint64_t cycles = 0;
    ++cycles;
    if (cycles == 1 || cycles % 100 == 0)
        LOG(INFO) << "[ReceiveDecoder] running, cycles=" << cycles;

    static auto output = context.get_channel_publisher<ECSData>(this);

    ECSData data{};
    data.my_color = MyColor::Blue;
    data.mode = AimMode::AutoAim;
    data.is_start = true;
    data.is_ready = false;
    output.publish(std::make_shared<const ECSData>(data));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

} // namespace app_plugin

REGISTER_PLUGIN("ReceiveDecoder", app_plugin::MinimalReceiveDecoder)
