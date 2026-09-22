#include "../config.hpp"
#include "_rcl/node.hpp"
#include "ascii_banner.hpp"
#include "backward-cpp/backward.hpp"
#include "param_deliver.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/semaphore_guard.hpp"
#include "utils/signal_guard.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <utility>

namespace backward {
static backward::SignalHandling sh;
}
using namespace awakening;
struct CameraTag {};
using CameraIO = IOPair<CameraTag, ImageFrame>;
int main(int argc, char** argv) {
    print_banner();
    auto& signal = utils::SignalGuard::instance();
    logger::init(spdlog::level::trace);
    auto get_arg = [&](int i) -> std::optional<std::string> {
        if (i < argc) {
            AWAKENING_INFO("get args {} ", std::string(argv[i]));
            return std::make_optional(std::string(argv[i]));
        }
        return std::nullopt;
    };
    std::string config_path;
    std::string robot_name;
    auto first_arg = get_arg(1);
    if (first_arg) {
        robot_name = first_arg.value();
        config_path = get_robot_config_path(robot_name).value_or(robot_name);
    } else {
        return 1;
    }
    auto config = YAML::LoadFile(config_path);
    Scheduler s;
    auto camera_config = config["camera"];
    std::unique_ptr<Camera> camera;
    utils::SignalGuard::add_callback([&]() {
        if (camera) {
            camera->stop();
        }
    });
    rcl::RclcppNode rcl_node("camera");
    camera = create_camera(camera_config, s, "hik");
    camera->init();
    if (!camera->is_running()) {
        return 0;
    }
    if (camera) {
        camera->start<CameraTag>("hik");
    }
    auto img_pub = rcl_node.make_pub<sensor_msgs::msg::Image>("image", rclcpp::QoS(10));
    s.register_task<CameraIO>("pub", [&](CameraIO::second_type&& f) {
        static std::unique_ptr<std::counting_semaphore<>> detector_sem;
        if (!detector_sem) {
            detector_sem = std::make_unique<std::counting_semaphore<>>(1);
        }
        {
            bool got = detector_sem->try_acquire();
            utils::SemaphoreGuard guard(*detector_sem, got);
            if (got) {
                cv::Mat src = f.src_img;
                sensor_msgs::msg::Image msg;
                msg.header.stamp = rcl_node.get_node()->now();
                msg.header.frame_id = "camera";

                // image size
                msg.height = src.rows;
                msg.width = src.cols;

                // OpenCV 默认 BGR
                msg.encoding = "bgr8";

                // x86/ARM 基本都是小端
                msg.is_bigendian = false;

                // 每行字节数
                msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(src.step);

                // 数据大小
                size_t size = src.step * src.rows;

                // 拷贝数据
                msg.data.resize(size);

                std::memcpy(msg.data.data(), src.data, size);
                // msg.data.data() = std::move(src.data);
                img_pub->publish(msg);
            }
        }
    });
    s.build();
    s.run();
    std::thread([&]() { rcl_node.spin(); }).detach();
    utils::SignalGuard::spin(std::chrono::milliseconds(1000));
    s.stop();
    rcl_node.shutdown();
    return 0;
}
