#include "tasks/sentry_brain/rmuc_2026/map.hpp"
#include "utils/logger.hpp"
#include <optional>
#include <string>
using namespace awakening::sentry_brain;
int main(int argc, char** argv) {
    awakening::logger::init(spdlog::level::trace);
    auto& map = RMUC2026Map::instance();
    auto second_arg = awakening::utils::get_arg(2, argc, argv);
    if (second_arg) {
        map.load_points_yaml(second_arg.value());
    }
    map.load_ros_map_yaml(awakening::utils::get_arg(1, argc, argv).value());
    map.visualize();
    map.dump_yaml(second_arg.value_or("output/rmuc_2026_map_point.yaml"));
}