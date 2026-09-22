#pragma once

#include "TrackerManager.hpp"

namespace app_plugin
{

class MinimalTrackerManager final : public ::TrackerManager
{
public:
    void process(const app::Context &context) override;
};

} // namespace app_plugin
