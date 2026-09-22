#include "NNDetector.hpp"

#include "class_loader.hpp"

#include <cstdint>
#include <glog/logging.h>

void NNDetector::process(const app::Context &context)
{
    static std::uint64_t cycles = 0;
    ++cycles;
    if (cycles == 1 || cycles % 100 == 0)
        LOG(INFO) << "[NNDetector] running, cycles=" << cycles;

    static auto input = context.get_buffer_subscriber<InputFrame>(this);
    static auto output = context.get_buffer_publisher<InputFrameWithNNResults>(this);

    InputFrameWithNNResults result;
    result.input_frame = input.wait_pop();
    output.push(std::move(result));
}

REGISTER_PLUGIN("NNDetector", NNDetector)
