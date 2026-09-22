#include "ascii_banner.hpp"
#include "tasks/base/common.hpp"
#include "tasks/radar_detect/rmuc_2026_map.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/signal_guard.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <thread>
#include <vector>

using namespace awakening;
struct CameraTag {};
struct SerialTag {};
struct DetectTag {};
struct FrameTag {};
using CameraIO = IOPair<CameraTag, ImageFrame>;
using SerialIO = IOPair<SerialTag, std::vector<uint8_t>>;
using CommonFrameIo = IOPair<FrameTag, CommonFrame>;
using DetIo = IOPair<DetectTag, std::vector<radar_detect::Cars>>;
int main(int argc, char** argv) {
    print_banner();
    auto& signal = utils::SignalGuard::instance();
    logger::init(spdlog::level::trace);

    std::string config_path;
    auto first_arg = utils::get_arg(1, argc, argv);
    if (first_arg) {
        config_path = first_arg.value();
    } else {
        return 1;
    }
    int cal_type = 0;
    auto second_arg = utils::get_arg(2, argc, argv);
    if (second_arg) {
        cal_type = std::stoi(second_arg.value());
    }
    auto config = YAML::LoadFile(config_path);
    if (cal_type == 1) {
        radar_detect::RMUC2026Map map(config["map"], radar_detect::SelfColor::RED);
        map.edit();
        map.dump_yaml("output/guess.yaml");
    } else {
        Scheduler s;
        std::unique_ptr<Camera> camera;
        auto camera_config = config["camera"];
        CameraInfo camera_info;
        camera_info.load(camera_config["camera_info"]);
        const std::string camera_type =
            utils::to_upper(camera_config["type"].as<std::string>("hik"));
        camera = create_camera(camera_config, s, "hik");
        if (camera_type != "VIDEO") {
            camera->init();
            if (!camera->is_running()) {
                return 0;
            }
        }
        cv::namedWindow("Image", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image", 640, 480);
        if (cal_type == 2) {
            s.register_task<CameraIO>("save_img", [&](CameraIO::second_type&& f) {
                cv::Mat img = f.src_img;
                cv::imshow("Image", img);
                auto key = cv::waitKey(1);
                if (key == 's') {
                    cv::imwrite("output/out.png", img);
                }
                return;
            });
        } else if (cal_type == 3) {
            struct BBoxState {
                cv::Mat current_img;
                cv::Rect2f current_box;
                bool drawing = false;
                bool has_box = false;
                cv::Point start_pt;

                std::vector<cv::Rect2f> boxes;
            };

            auto state = std::make_shared<BBoxState>();

            cv::setMouseCallback(
                "Image",
                [](int event, int x, int y, int, void* userdata) {
                    auto* s = static_cast<BBoxState*>(userdata);

                    if (event == cv::EVENT_LBUTTONDOWN) {
                        s->drawing = true;
                        s->start_pt = cv::Point(x, y);
                        s->current_box = cv::Rect2f(x, y, 0, 0);
                    }

                    else if (event == cv::EVENT_MOUSEMOVE && s->drawing)
                    {
                        int x0 = std::min(s->start_pt.x, x);
                        int y0 = std::min(s->start_pt.y, y);
                        int w = std::abs(x - s->start_pt.x);
                        int h = std::abs(y - s->start_pt.y);

                        s->current_box = cv::Rect2f(x0, y0, w, h);
                    }

                    else if (event == cv::EVENT_LBUTTONUP)
                    {
                        s->drawing = false;

                        int x0 = std::min(s->start_pt.x, x);
                        int y0 = std::min(s->start_pt.y, y);
                        int w = std::abs(x - s->start_pt.x);
                        int h = std::abs(y - s->start_pt.y);

                        s->current_box = cv::Rect2f(x0, y0, w, h);

                        if (w > 5 && h > 5) {
                            s->boxes.push_back(s->current_box);

                            AWAKENING_INFO(
                                "BBox: x={} y={} w={} h={}",
                                s->current_box.x,
                                s->current_box.y,
                                s->current_box.width,
                                s->current_box.height
                            );
                        }
                    }
                },
                state.get()
            );

            s.register_task<CameraIO>("get_bbox", [state](CameraIO::second_type&& f) {
                state->current_img = f.src_img.clone();

                cv::Mat show = state->current_img.clone();

                // 已确认 bbox
                for (auto& box: state->boxes) {
                    cv::rectangle(show, box, cv::Scalar(0, 255, 0), 2);
                }

                // 正在拖拽 bbox
                if (state->drawing) {
                    cv::rectangle(show, state->current_box, cv::Scalar(0, 0, 255), 2);
                }

                cv::imshow("Image", show);

                auto key = cv::waitKey(1);

                // 保存截图
                if (key == 's') {
                    cv::imwrite("bbox_image.png", state->current_img);

                    YAML::Node node;

                    for (size_t i = 0; i < state->boxes.size(); ++i) {
                        auto& b = state->boxes[i];

                        YAML::Node item;
                        item["x"] = b.x;
                        item["y"] = b.y;
                        item["w"] = b.width;
                        item["h"] = b.height;

                        node["bboxes"].push_back(item);
                    }

                    std::ofstream fout("output/bbox.yaml");
                    fout << node;

                    AWAKENING_INFO("Saved output/bbox_image.png and bbox.yaml");
                }

                // 清空
                if (key == 'c') {
                    state->boxes.clear();
                    AWAKENING_INFO("Clear bboxes");
                }

                return;
            });
        }
        if (camera) {
            camera->start<CameraTag>(camera_type == "VIDEO" ? "video" : "hik");
        }
        s.build();
        s.run();
        utils::SignalGuard::spin(std::chrono::milliseconds(1000));
        s.stop();
    }

    return 0;
}
