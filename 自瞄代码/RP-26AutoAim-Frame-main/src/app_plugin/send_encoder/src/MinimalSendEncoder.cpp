#include "MinimalSendEncoder.hpp"

#include "class_loader.hpp"

#include <cstdint>
#include <glog/logging.h>

void app_plugin::MinimalSendEncoder::process(const app::Context &context)
{
    static std::uint64_t cycles = 0;
    ++cycles;
    if (cycles == 1 || cycles % 100 == 0)
        LOG(INFO) << "[SendEncoder] running, cycles=" << cycles;

    static auto input = context.get_buffer_subscriber<FireResult>(this);
    (void)input.wait_pop();
}

REGISTER_PLUGIN("SendEncoder", app_plugin::MinimalSendEncoder)
