#pragma once

#include "SendEncoder.hpp"

namespace app_plugin
{

class MinimalSendEncoder final : public SendEncoder
{
public:
    void process(const app::Context &context) override;
};

} // namespace app_plugin
