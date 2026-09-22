#pragma once
#include "context.hpp"
#include "type/ECSData.hpp"
#include "type/InputFrame.hpp"

class CameraManager : public app::PluginBase
{
public:
    void declare(app::Context &context) override;

};

inline void CameraManager::declare(app::Context &context)
{
    context.declare_input_channel<ECSData>(this);
    context.declare_output_buffer<InputFrame>(this);
}
