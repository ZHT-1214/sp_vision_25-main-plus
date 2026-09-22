#pragma once
#include "context.hpp"
#include "type/TrackResult.hpp"
#include "type/ECSData.hpp"
#include "type/FireResult.hpp"

class FireControlSystem : public app::PluginBase
{
public:
    void declare(app::Context &context) override;
};

inline void FireControlSystem::declare(app::Context &context)
{
    context.declare_input_channel<ECSData>(this);
    context.declare_input_buffer<TrackResult>(this);
    context.declare_output_buffer<FireResult>(this);
}
