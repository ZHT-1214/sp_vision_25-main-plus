#include "utils/drivers/camera_factory.hpp"

#include "utils/logger.hpp"
#include "utils/utils.hpp"

#include <stdexcept>

#ifdef USE_DahengSDK
    #include "utils/drivers/daheng_camera.hpp"
#endif
#ifdef USE_HikSDK
    #include "utils/drivers/hik_camera.hpp"
#endif
#ifdef USE_MvSDK
    #include "utils/drivers/mv_camera.hpp"
#endif
#include "utils/drivers/uvc_camera.hpp"
#include "utils/drivers/video_player.hpp"

namespace awakening {

namespace {

    std::string camera_type_from_config(const YAML::Node& config, const std::string& default_type) {
        return utils::to_upper(config["type"].as<std::string>(default_type));
    }

    [[noreturn]] void throw_unsupported_camera(const std::string& type) {
        throw std::runtime_error("unsupported camera type: " + type);
    }

} // namespace

std::unique_ptr<Camera>
create_camera(const YAML::Node& config, Scheduler& scheduler, const std::string& default_type) {
    const auto type = camera_type_from_config(config, default_type);

    if (type == "VIDEO" || type == "VIDEO_PLAYER") {
        return std::make_unique<VideoPlayer>(config["video"], scheduler);
    }
    if (type == "UVC") {
        return std::make_unique<UVCCamera>(config["uvc_camera"], &scheduler);
    }
    if (type == "HIK" || type == "HIK_CAMERA") {
#ifdef USE_HikSDK
        return std::make_unique<HikCamera>(config["hik_camera"], scheduler);
#else
        AWAKENING_ERROR("Hik camera requested but USE_HikSDK is disabled");
        throw_unsupported_camera(type);
#endif
    }
    if (type == "MV" || type == "MV_CAMERA") {
#ifdef USE_MvSDK
        return std::make_unique<MvCamera>(config["mv_camera"], scheduler);
#else
        AWAKENING_ERROR("Mv camera requested but USE_MvSDK is disabled");
        throw_unsupported_camera(type);
#endif
    }
    if (type == "DAHENG" || type == "DAHENG_CAMERA") {
#ifdef USE_DahengSDK
        return std::make_unique<DahengCamera>(config["daheng_camera"], scheduler);
#else
        AWAKENING_ERROR("Daheng camera requested but USE_DahengSDK is disabled");
        throw_unsupported_camera(type);
#endif
    }

    throw_unsupported_camera(type);
}

std::unique_ptr<Camera> create_camera(const YAML::Node& config, const std::string& default_type) {
    const auto type = camera_type_from_config(config, default_type);

    if (type == "UVC") {
        return std::make_unique<UVCCamera>(config["uvc_camera"]);
    }

    throw_unsupported_camera(type);
}

} // namespace awakening
