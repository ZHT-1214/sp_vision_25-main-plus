#include "utils/utils.hpp"
#include <optional>
#include <string>
#define DEFINE_CONFIG_PATH(NAME, FILE) \
    static constexpr auto NAME##_ARR = utils::concat(ROOT_DIR, FILE); \
    static constexpr std::string_view NAME(NAME##_ARR.data());
namespace awakening {
DEFINE_CONFIG_PATH(OMNI_CONFIG_PATH, "/config/omni.yaml")
DEFINE_CONFIG_PATH(SENTRY_CONFIG_PATH, "/config/sentry.yaml")
DEFINE_CONFIG_PATH(LEG_CONFIG_PATH, "/config/leg.yaml")
DEFINE_CONFIG_PATH(HERO_CONFIG_PATH, "/config/hero.yaml")
DEFINE_CONFIG_PATH(UAV_CONFIG_PATH, "/config/uav.yaml")
DEFINE_CONFIG_PATH(DONG_CONFIG_PATH, "/config/dong.yaml")
DEFINE_CONFIG_PATH(RMUC_2026_POINT_CONFIG_PATH, "/config/rmuc_2026_map_point.yaml")
inline std::optional<std::string> get_robot_config_path(std::string name) {
    auto key = utils::to_upper(name);
    if (key == "OMNI") {
        return std::string(OMNI_CONFIG_PATH);
    } else if (key == "SENTRY") {
        return std::string(SENTRY_CONFIG_PATH);
    } else if (key == "LEG") {
        return std::string(LEG_CONFIG_PATH);
    } else if (key == "HERO") {
        return std::string(HERO_CONFIG_PATH);
    } else if (key == "UAV") {
        return std::string(UAV_CONFIG_PATH);
    } else if (key == "DONG") {
        return std::string(DONG_CONFIG_PATH);
    }
    return std::nullopt;
}
} // namespace awakening
