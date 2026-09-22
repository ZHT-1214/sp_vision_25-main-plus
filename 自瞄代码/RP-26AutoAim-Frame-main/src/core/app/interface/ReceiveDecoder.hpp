#pragma once
#include "context.hpp"
#include "type/ECSData.hpp"

class ReceiveDecoder : public app::PluginBase
{
public:
    void declare(app::Context &context) override;
};

inline void ReceiveDecoder::declare(app::Context &context)
{
    context.declare_output_channel<ECSData>(this);
}
