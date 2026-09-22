#pragma once
#include "context.hpp"
#include "type/FireResult.hpp"

class SendEncoder : public app::PluginBase
{
public:
    void declare(app::Context &context) override;
};

inline void SendEncoder::declare(app::Context &context)
{
    context.declare_input_buffer<FireResult>(this);
}
