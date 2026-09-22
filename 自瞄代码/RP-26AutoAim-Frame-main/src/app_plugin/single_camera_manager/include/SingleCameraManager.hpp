#pragma once

#include "CameraManager.hpp"

namespace app_plugin
{

// Creates an empty frame without opening a camera.
class SingleCameraManager final : public CameraManager
{
public:
    void declare(app::Context &context) override
    {
        CameraManager::declare(context);
    }

    void process(const app::Context &context) override;
};

} // namespace app_plugin
