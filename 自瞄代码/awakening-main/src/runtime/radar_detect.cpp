#include "_rcl/tf.hpp"
#include "ascii_banner.hpp"
#include "tasks/radar_detect/car_pool.hpp"
#include "tasks/radar_detect/detector.hpp"
#include "tasks/radar_detect/pixel_to_world.hpp"
#include "tasks/radar_io/frame.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/drivers/serial_driver.hpp"
#include "utils/io/video_save.hpp"
#include "utils/semaphore_guard.hpp"
#include "utils/signal_guard.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/node/parse.h>
using namespace awakening;
struct CameraTag {};
struct DetectTag {};
struct FrameTag {};
using CameraIO = IOPair<CameraTag, ImageFrame>;
using CommonFrameIo = IOPair<FrameTag, CommonFrame>;
using DetIo = IOPair<DetectTag, std::vector<radar_detect::Cars>>;

struct RefereeSerialTag {};
using RefereeSerialIO = IOPair<RefereeSerialTag, std::vector<uint8_t>>;
struct WifiSerialTag {};
using WifiSerialIO = IOPair<WifiSerialTag, std::vector<uint8_t>>;
enum class RadarFrame : int { TARGET_MAP, CAMERA_CV, N };
using RadarTF = utils::tf::RobotTF<RadarFrame, static_cast<size_t>(RadarFrame::N), false>;
std::string RadarFrame_to_str(int f) {
    constexpr const char* details[] = { "target_map", "camera_cv" };
    return std::string(details[f]);
}
std::string RadarFrame_to_str(RadarFrame f) {
    return RadarFrame_to_str(std::to_underlying(f));
}
void draw_star(cv::Mat& img, cv::Point center, int radius, cv::Scalar color, int thickness = -1) {
    const int num_points = 5;
    std::vector<cv::Point> pts(2 * num_points);

    double angle = -CV_PI / 2; // 从顶点开始
    double delta = CV_PI / num_points;

    for (int i = 0; i < 2 * num_points; ++i) {
        double r = (i % 2 == 0) ? radius : radius / 2.5;
        pts[i] = cv::Point(
            static_cast<int>(center.x + r * cos(angle)),
            static_cast<int>(center.y + r * sin(angle))
        );
        angle += delta;
    }

    cv::fillPoly(img, std::vector<std::vector<cv::Point>> { pts }, color);
}
struct LogCtx {
    int image_count;
    int detect_count;
    double detect_cost_ms;
    int receive_referee_count;
    int receive_wifi_count;
    void reset() {
        image_count = 0;
        detect_count = 0;
        detect_cost_ms = 0;
        receive_referee_count = 0;
        receive_wifi_count = 0;
    }
};
static constexpr auto RECORD_FOLDER_PATH_ARR = utils::concat(ROOT_DIR, "/record/radar");
static constexpr std::string_view RECORD_FOLDER_PATH(RECORD_FOLDER_PATH_ARR.data());

int main(int argc, char** argv) {
    print_banner();
    auto& signal = utils::SignalGuard::instance();
    logger::init(spdlog::level::trace);

    bool debug = false;
    std::string config_path;
    auto first_arg = utils::get_arg(1, argc, argv);
    if (first_arg) {
        config_path = first_arg.value();
    } else {
        return 1;
    }
    LogCtx log_ctx;
    auto config = YAML::LoadFile(config_path);
    auto camera_config = config["camera"];
    CameraInfo camera_info;
    camera_info.load(camera_config["camera_info"]);
    Scheduler s;
    rcl::RclcppNode rcl_node("radar_detect");
    rcl::TF rcl_tf(rcl_node);
    RadarTF tf;
    {
        tf.add_edge(RadarFrame::TARGET_MAP, RadarFrame::CAMERA_CV);
        ISO3 camera_cv_in_target_map =
            utils::load_isometry3(config["tf"]["camera_cv_in_target_map"]);
        tf.push(
            RadarFrame::TARGET_MAP,
            RadarFrame::CAMERA_CV,
            Clock::now(),
            camera_cv_in_target_map
        );
    }
    std::unique_ptr<Camera> camera;
    std::unique_ptr<VideoSaver> video_saver;
    utils::SignalGuard::add_callback([&]() {
        if (camera) {
            camera->stop();
        }
    });
    std::string camera_type = utils::to_upper(camera_config["type"].as<std::string>("hik"));
    camera = create_camera(camera_config, s, "hik");
    if (camera_type != "VIDEO") {
        video_saver = std::make_unique<VideoSaver>(
            VideoSaver::generate_record_filename(std::string(RECORD_FOLDER_PATH))
        );
        camera->init();
        if (!camera->is_running()) {
            return 0;
        }
    }
    std::unique_ptr<SerialDriver> wifi_serial;
    if (config["wifi_serial"]["enable"].as<bool>()) {
        wifi_serial = std::make_unique<SerialDriver>(config["wifi_serial"], s);
    }
    std::unique_ptr<SerialDriver> referee_serial;
    if (config["referee_serial"]["enable"].as<bool>()) {
        referee_serial = std::make_unique<SerialDriver>(config["referee_serial"], s);
    }
    bool enemy_outpost_active = false;
    utils::OrderedQueue<radar_detect::Cars> cars_queue;
    radar_detect::SelfColor self_color =
        radar_detect::SelfColor_from_str(config["self_color"].as<std::string>());
    radar_detect::Detector detector(config["detector"]);
    radar_detect::Tracker tracker(config["tracker"]);
    radar_detect::RMUC2026Map map(config["map"], self_color);
    utils::SWMR<std::unordered_map<int, radar_detect::TheOnlyCar>> fin_cars;
    radar_detect::CarPool car_pool(config["car_pool"], map);
    fin_cars.write(car_pool.get_fin_cars());
    ISO3 camera_cv_in_target_map =
        tf.pose_a_in_b(RadarFrame::CAMERA_CV, RadarFrame::TARGET_MAP, Clock::now());
    radar_detect::PixelToWorld pixel_to_world(
        config["pixel_to_world"],
        camera_cv_in_target_map,
        camera_info
    );
    radar_detect::RadarDebugCtx debug_ctx;
    debug_ctx.map = map.image.clone();
    auto outpost_bbox = utils::load_rect2f(config["outpost_bbox"]);
    std::vector<cv::Point2f> cal_pts;
    for (const auto& pt: config["cal_pts"]) {
        if (pt.size() != 2) {
            std::cerr << "每个点必须有两个元素" << std::endl;
            continue;
        }
        float x = pt[0].as<float>();
        float y = pt[1].as<float>();

        cal_pts.emplace_back(x, y);
    }
    s.register_task<CameraIO, CommonFrameIo>("push_common_frame", [&](CameraIO::second_type&& f) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        static int current_id = 0;
        CommonFrame frame {
            .img_frame = std::move(f),
            .id = current_id++,
            .frame_id = 0,
        };
        log_ctx.image_count++;
        if (video_saver) {
            video_saver->write_frame(f.src_img);
        }
        return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::move(frame)));
    });

    s.register_task<CommonFrameIo, DetIo>("detector", [&](CommonFrameIo::second_type&& frame) {
        static std::unique_ptr<std::counting_semaphore<>> detector_sem;
        if (!detector_sem) {
            detector_sem =
                std::make_unique<std::counting_semaphore<>>(config["max_infer_num"].as<int>());
        }
        static std::vector<radar_detect::Armor> outpost_armors;
        auto img = frame.img_frame.src_img;

        radar_detect::Cars cars { .t = frame.img_frame.timestamp, .id = frame.id };
        {
            bool got = detector_sem->try_acquire();
            utils::SemaphoreGuard guard(*detector_sem, got);
            if (got) {
                log_ctx.detect_count++;
                auto start = Clock::now();
                cars.cars = detector.detect(
                    frame,
                    cv::Rect(0, 0, frame.img_frame.src_img.cols, frame.img_frame.src_img.rows)
                );
                utils::dt_once(
                    [&]() {
                        CommonFrame f = frame;
                        outpost_armors = detector.detect_armors(f, outpost_bbox);
                        enemy_outpost_active = false;
                        for (const auto& o: outpost_armors) {
                            if (o.number == radar_detect::ArmorClass::OUTPOST) {
                                enemy_outpost_active = true;
                            }
                        }
                    },
                    std::chrono::duration<double>(1.0)
                );
                auto end = Clock::now();
                log_ctx.detect_cost_ms +=
                    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            }
        }

        std::vector<radar_detect::Car> valid_cars;
        for (auto& car: cars.cars) {
            auto p_opt = pixel_to_world.pixel_to_world(car.get_key_point());
            if (p_opt) {
                car.point_in_uwb = p_opt.value();
                if (self_color == radar_detect::SelfColor::BLUE) {
                    car.point_in_uwb.x() = radar_detect::FIELD_LONGTH - car.point_in_uwb.x();
                    car.point_in_uwb.y() = radar_detect::FIELD_WIDTH - car.point_in_uwb.y();
                }

                valid_cars.push_back(car);
            }
        }
        debug_ctx.cars.set(cars.cars);
        debug_ctx.outpost.set(outpost_armors);
        cars_queue.enqueue(cars);
        auto batch_cars = cars_queue.dequeue_batch();

        debug_ctx.img_frame.set(std::move(frame.img_frame));
        return std::make_tuple(std::optional<DetIo::second_type>(std::move(batch_cars)));
    });
    s.register_task<DetIo>("tracker", [&](DetIo::second_type&& io) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& cars: io) {
            tracker.update(cars.cars, cars.t);
            car_pool.update(tracker.get_targets());
        }
        auto _fin_cars = car_pool.get_fin_cars();
        fin_cars.write(_fin_cars);
    });
    cv::namedWindow("Video Frame", cv::WINDOW_NORMAL);
    cv::resizeWindow("Video Frame", 800, 600);
    cv::namedWindow("Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Map", 800, 600);
    auto get_cv_color_from_car = [&](radar_detect::CarClass c) {
        int k = std::to_underlying(c);
        if (k < 0) {
            return cv::Scalar(0, 255, 0);
        } else if (k < 100) {
            return cv::Scalar(0, 0, 255);
        } else {
            return cv::Scalar(255, 0, 0);
        }
    };
    s.add_rate_source<>("debug", 10.0, [&]() {
        auto image = debug_ctx.img_frame.get().src_img;
        if (image.empty()) {
            return;
        }
        auto cars = debug_ctx.cars.get();
        auto outpost = debug_ctx.outpost.get();
        auto map = debug_ctx.map.clone();
        if (map.image.empty()) {
            return;
        }
        cv::resize(map.image, map.image, cv::Size(map.image.cols * 3.0, map.image.rows * 3.0));
        auto _fin_cars = fin_cars.read();
        for (const auto& car: cars) {
            car.draw(image);
        }
        for (const auto& o: outpost) {
            o.draw(image);
        }
        for (const auto& pt: cal_pts) {
            cv::circle(image, pt, 20, cv::Scalar(255, 0, 255), -1);
        }
        cv::rectangle(image, outpost_bbox, cv::Scalar(0, 255, 0), 2);
        for (const auto& car: _fin_cars) {
            auto img_point =
                radar_detect::uwb_to_image(map, car.second.uwb_state.state.pos().head<2>());
            cv::putText(
                map.image,
                radar_detect::CarClass_to_str(car.second.car_class),
                img_point,
                cv::FONT_HERSHEY_SIMPLEX,
                3.0,
                get_cv_color_from_car(car.second.car_class),
                2
            );
            auto vel_end = radar_detect::uwb_to_image(
                map,
                car.second.uwb_state.state.pos().head<2>()
                    + car.second.uwb_state.state.vel().head<2>()
            );
            {
                cv::Point start = img_point;
                cv::Point end = vel_end;

                // 箭头长度
                double length = cv::norm(end - start);
                if (length > 1.0) { // 防止速度过小箭头不可见
                    double tip_ratio =
                        std::min(0.3, 0.2 + 0.1 * (length / 50.0)); // tip 长度随箭头长度变化
                    cv::arrowedLine(
                        map.image,
                        start,
                        end,
                        get_cv_color_from_car(car.second.car_class),
                        2, // 线宽
                        cv::LINE_AA, // 抗锯齿
                        0,
                        tip_ratio // 自适应箭头头部
                    );
                }
            }
            double r = 0.6 / radar_detect::FIELD_LONGTH * map.image.size().width;
            if (car.second.car_state != radar_detect::CarState::GUESSING) {
                cv::circle(
                    map.image,
                    img_point,
                    30,
                    get_cv_color_from_car(car.second.car_class),
                    -1
                );
            } else {
                draw_star(map.image, img_point, 30, get_cv_color_from_car(car.second.car_class));
            }

            cv::circle(map.image, img_point, r, get_cv_color_from_car(car.second.car_class), 5);
        }
        cv::imshow("Map", map.image);
        cv::imshow("Video Frame", image);
        cv::waitKey(1);
    });
    radar_io::RadarInfo radar_info;
    radar_io::RadarCmd radar_cmd;
    radar_io::MapRobotData map_robot_data;
    radar_io::ToSenrty to_sentry;
    auto parse_referee = [&](uint16_t cmd_id, uint8_t* data, size_t len) {
        log_ctx.receive_referee_count++;
        auto data_vec = std::vector<uint8_t>(data, data + len);
        auto cmd = radar_io::CMDID(cmd_id);
        switch (cmd) {
            case radar_io::CMDID::RoboStatus: {
                auto r = utils::from_vector<radar_io::RoboStatus>(data_vec);
                int robot_id = r.robot_id;
                if (robot_id != 9 && robot_id != 109) {
                    AWAKENING_WARN("i am not radar!");
                }
                if (robot_id < 50) {
                    self_color = radar_detect::SelfColor::RED;
                } else {
                    self_color = radar_detect::SelfColor::BLUE;
                }
                break;
            }
            case radar_io::CMDID::RadarMark: {
                auto m = radar_io::RadarMark::create(data, len);
                break;
            }
            case radar_io::CMDID::RadarInfo: {
                radar_info = radar_io::RadarInfo::create(data, len);
                break;
            }
        }
    };
    if (referee_serial) {
        s.register_task<RefereeSerialIO>(
            "receive_referee_serial",
            [&](RefereeSerialIO::second_type&& data) {
                static std::deque<uint8_t> rx_buffer;
                rx_buffer.insert(rx_buffer.end(), data.begin(), data.end());

                while (true) {
                    if (rx_buffer.size() < 5)
                        return;

                    while (!rx_buffer.empty() && rx_buffer.front() != 0xA5) {
                        rx_buffer.pop_front();
                    }

                    if (rx_buffer.size() < 5)
                        return;

                    radar_io::FrameHeader header;

                    header.sof = rx_buffer[0];
                    header.data_length = rx_buffer[1] | (rx_buffer[2] << 8);

                    header.seq = rx_buffer[3];
                    header.crc8 = rx_buffer[4];
                    if (!radar_io::verify_crc8(&rx_buffer[0], static_cast<uint32_t>(5))) {
                        rx_buffer.pop_front();
                        AWAKENING_WARN("crc8 failed");
                        continue;
                    }
                    size_t full_len = 5 + // frame_header
                        2 + // cmd_id
                        header.data_length + 2; // crc16
                    if (rx_buffer.size() < full_len)
                        return;

                    std::vector<uint8_t> frame(rx_buffer.begin(), rx_buffer.begin() + full_len);
                    if (!radar_io::verify_crc16(frame.data(), full_len)) {
                        rx_buffer.pop_front();
                        AWAKENING_WARN("crc16 failed");
                        continue;
                    }

                    uint16_t cmd_id = frame[5] | (frame[6] << 8);
                    uint8_t* payload = frame.data() + 7;
                    parse_referee(cmd_id, payload, static_cast<uint32_t>(header.data_length));

                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + full_len);
                }
            }
        );
    }
    radar_io::FromWifi from_wifi;
    radar_io::ToWifi to_wifi;
    bool rf_key_right = false;
    TimePoint rf_info_time = Clock::now();
    if (wifi_serial) {
        s.register_task<WifiSerialIO>("receive_wifi_serial", [&](WifiSerialIO::second_type&& data) {
            auto wifi_opt = radar_io::FromWifi::create(data);
            if (wifi_opt) {
                auto wifi = wifi_opt.value();
                static uint32_t rf_info_count = 0;
                static uint32_t rf_jam_count = 0;
                if (wifi.rf_info_count > rf_info_count) {
                    rf_info_count = wifi.rf_info_count;
                    rf_key_right = true;
                    AWAKENING_INFO("receive ok rf key");
                }
                if (wifi.rf_jam_count > rf_jam_count) {
                    rf_jam_count = wifi.rf_jam_count;
                    rf_info_time = Clock::now();
                    AWAKENING_INFO("receive ok rf info");
                }
                log_ctx.receive_wifi_count++;
            }
        });
    }

    s.add_rate_source<>("main", 30.0, [&]() {
        auto _fin_cars = fin_cars.read();
        auto msg = radar_detect::CarPool::to_msg(self_color, _fin_cars);
        map_robot_data.opponent_hero_position_x = msg.enemy_no1_x;
        map_robot_data.opponent_hero_position_y = msg.enemy_no1_y;
        map_robot_data.opponent_engineer_position_x = msg.enemy_no2_x;
        map_robot_data.opponent_engineer_position_y = msg.enemy_no2_y;
        map_robot_data.opponent_infantry_3_position_x = msg.enemy_no3_x;
        map_robot_data.opponent_infantry_3_position_y = msg.enemy_no3_y;
        map_robot_data.opponent_infantry_4_position_x = msg.enemy_no4_x;
        map_robot_data.opponent_infantry_4_position_y = msg.enemy_no4_y;
        map_robot_data.opponent_aerial_position_x = msg.enemy_no6_x;
        map_robot_data.opponent_aerial_position_y = msg.enemy_no6_y;
        map_robot_data.opponent_sentry_position_x = msg.enemy_no7_x;
        map_robot_data.opponent_sentry_position_y = msg.enemy_no7_y;
        if (std::chrono::duration<double>(Clock::now() - rf_info_time)
            < std::chrono::duration<double>(1.0)) {
            // map_robot_data.opponent_hero_position_x = from_wifi.RF_Position_Struct.Robo_1_X_cm;
            // map_robot_data.opponent_hero_position_y = from_wifi.RF_Position_Struct.Robo_1_Y_cm;
            // map_robot_data.opponent_engineer_position_x = from_wifi.RF_Position_Struct.Robo_2_X_cm;
            // map_robot_data.opponent_engineer_position_y = from_wifi.RF_Position_Struct.Robo_2_Y_cm;
            // map_robot_data.opponent_infantry_3_position_x =
            //     from_wifi.RF_Position_Struct.Robo_3_X_cm;
            // map_robot_data.opponent_infantry_3_position_y =
            //     from_wifi.RF_Position_Struct.Robo_3_Y_cm;
            // map_robot_data.opponent_infantry_4_position_x =
            //     from_wifi.RF_Position_Struct.Robo_4_X_cm;
            // map_robot_data.opponent_infantry_4_position_y =
            //     from_wifi.RF_Position_Struct.Robo_4_Y_cm;
            // map_robot_data.opponent_aerial_position_x = from_wifi.RF_Position_Struct.Robo_6_X_cm;
            // map_robot_data.opponent_aerial_position_y = from_wifi.RF_Position_Struct.Robo_6_Y_cm;
            // map_robot_data.opponent_sentry_position_x = from_wifi.RF_Position_Struct.Robo_5_X_cm;
            // map_robot_data.opponent_sentry_position_y = from_wifi.RF_Position_Struct.Robo_5_Y_cm;
        }
        map_robot_data.ally_hero_position_x = msg.self_no1_x;
        map_robot_data.ally_hero_position_y = msg.self_no1_y;
        map_robot_data.ally_engineer_position_x = msg.self_no2_x;
        map_robot_data.ally_engineer_position_y = msg.self_no2_y;
        map_robot_data.ally_infantry_3_position_x = msg.self_no3_x;
        map_robot_data.ally_infantry_3_position_y = msg.self_no3_y;
        map_robot_data.ally_infantry_4_position_x = msg.self_no4_x;
        map_robot_data.ally_infantry_4_position_y = msg.self_no4_y;
        map_robot_data.ally_aerial_position_x = msg.self_no6_x;
        map_robot_data.ally_aerial_position_y = msg.self_no6_y;
        map_robot_data.ally_sentry_position_x = msg.self_no7_x;
        map_robot_data.ally_sentry_position_y = msg.self_no7_y;
        if (enemy_outpost_active) {
            to_sentry.enemy_outpost_active = enemy_outpost_active;
        }
        to_sentry.opponent_hero_position_x = map_robot_data.opponent_hero_position_x;
        to_sentry.opponent_hero_position_y = map_robot_data.opponent_hero_position_y;
        to_sentry.opponent_engineer_position_x = map_robot_data.opponent_engineer_position_x;
        to_sentry.opponent_engineer_position_y = map_robot_data.opponent_engineer_position_y;
        to_sentry.opponent_infantry_3_position_x = map_robot_data.opponent_infantry_3_position_x;
        to_sentry.opponent_infantry_3_position_y = map_robot_data.opponent_infantry_3_position_y;
        to_sentry.opponent_infantry_4_position_x = map_robot_data.opponent_infantry_4_position_x;
        to_sentry.opponent_infantry_4_position_y = map_robot_data.opponent_infantry_4_position_y;
        to_sentry.opponent_aerial_position_x = map_robot_data.opponent_aerial_position_x;
        to_sentry.opponent_aerial_position_y = map_robot_data.opponent_aerial_position_y;
        to_sentry.opponent_sentry_position_x = map_robot_data.opponent_sentry_position_x;
        to_sentry.opponent_sentry_position_y = map_robot_data.opponent_sentry_position_y;
        auto _to_sentry = radar_io::RobotInteractionData::create(
            self_color == radar_detect::SelfColor::RED ? std::to_underlying(radar_io::RoboID::R9)
                                                       : std::to_underlying(radar_io::RoboID::B9),

            self_color == radar_detect::SelfColor::RED ? std::to_underlying(radar_io::RoboID::R7)
                                                       : std::to_underlying(radar_io::RoboID::B7),
            to_sentry
        );
        radar_io::CustomInfo to_no1;
        to_no1.sender_id = self_color == radar_detect::SelfColor::RED
            ? std::to_underlying(radar_io::RoboID::R9)
            : std::to_underlying(radar_io::RoboID::B9);
        to_no1.receiver_id = self_color == radar_detect::SelfColor::RED
            ? std::to_underlying(radar_io::RoboID::R2OP)
            : std::to_underlying(radar_io::RoboID::B2OP);
        constexpr size_t max_bytes = sizeof(to_no1.user_data);

        std::u16string s = u"aaa";
        std::memset(to_no1.user_data, 0, max_bytes);
        size_t copy_bytes = std::min(s.size() * sizeof(char16_t), max_bytes - 2);
        std::memcpy(to_no1.user_data, s.data(), copy_bytes);
        static uint8_t double_vulnerability_count = 0;
        double_vulnerability_count = (double_vulnerability_count + 1) % 3;
        radar_cmd.radar_cmd = double_vulnerability_count;
        radar_cmd.password_cmd = 2;
        radar_cmd.password_1 = 'r';
        radar_cmd.password_2 = 'm';
        radar_cmd.password_3 = '2';
        radar_cmd.password_4 = '0';
        radar_cmd.password_5 = '2';
        radar_cmd.password_6 = '6';
        if (rf_key_right) {
            radar_cmd.password_1 = from_wifi.RF_Key_Struct.Key[0];
            radar_cmd.password_2 = from_wifi.RF_Key_Struct.Key[1];
            radar_cmd.password_3 = from_wifi.RF_Key_Struct.Key[2];
            radar_cmd.password_4 = from_wifi.RF_Key_Struct.Key[3];
            radar_cmd.password_5 = from_wifi.RF_Key_Struct.Key[4];
            radar_cmd.password_6 = from_wifi.RF_Key_Struct.Key[5];
        }
        if (radar_info.can_change_key) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<uint16_t> dis(0, 255);
            radar_cmd.password_cmd = 1;
            radar_cmd.password_1 = static_cast<uint8_t>(dis(gen));
            radar_cmd.password_2 = static_cast<uint8_t>(dis(gen));
            radar_cmd.password_3 = static_cast<uint8_t>(dis(gen));
            radar_cmd.password_4 = static_cast<uint8_t>(dis(gen));
            radar_cmd.password_5 = static_cast<uint8_t>(dis(gen));
            radar_cmd.password_6 = static_cast<uint8_t>(dis(gen));
        }
        auto _radar_cmd = radar_io::RobotInteractionData::create(
            self_color == radar_detect::SelfColor::RED ? 9 : 109,
            std::to_underlying(radar_io::RoboID::REFEREE),
            radar_cmd
        );
        to_wifi.cmd_id = radar_io::ToWifi::CMDID;
        to_wifi.robot_id = self_color == radar_detect::SelfColor::RED ? 9 : 109;
        to_wifi.jam_level = radar_info.encryption_level;
        if (referee_serial) {
            if (!referee_serial->write(radar_io::pack_frame(map_robot_data))) {
                AWAKENING_ERROR("FUCK");
            }
            if (!referee_serial->write(radar_io::pack_frame(to_no1))) {
                AWAKENING_ERROR("FUCK");
            }
            if (!referee_serial->write(radar_io::pack_frame(_radar_cmd))) {
                AWAKENING_ERROR("FUCK");
            }
            if (!referee_serial->write(radar_io::pack_frame(_to_sentry))) {
                AWAKENING_ERROR("FUCK");
            }
        }
        if (wifi_serial) {
            wifi_serial->write(utils::to_vector(to_wifi));
        }
    });
    if (camera) {
        camera->start<CameraTag>(camera_type == "VIDEO" ? "video" : "hik");
    }
    if (referee_serial) {
        referee_serial->start<RefereeSerialTag>("referee_serial");
    }
    if (wifi_serial) {
        wifi_serial->start<WifiSerialTag>("wifi");
    }
    s.add_rate_source<>("tf_pub", 100.0, [&]() {
        rcl_tf.pub_robot_tf(tf, [](RadarFrame frame) { return RadarFrame_to_str(frame); });
    });
    s.add_rate_source<>("log", 1.0, [&]() {
        AWAKENING_INFO(
            "img: {}, det: {},avg_cost: {:.2f}ms, referee: {}, wifi: {}, self:{}",
            log_ctx.image_count,
            log_ctx.detect_count,
            log_ctx.detect_cost_ms / (log_ctx.detect_count ? log_ctx.detect_count : 1),
            log_ctx.receive_referee_count,
            log_ctx.receive_wifi_count,
            self_color == radar_detect::SelfColor::RED ? "red" : "blue"
        );
        log_ctx.reset();
    });
    s.build();
    s.run();
    std::thread([&]() { rcl_node.spin(); }).detach();

    utils::SignalGuard::spin(std::chrono::milliseconds(1000));

    s.stop();
    rcl_node.shutdown();
    cv::destroyAllWindows();

    for (int i = 0; i < 10; ++i) {
        AWAKENING_CRITICAL("改了东西记得同步其他有关的exe的src");
    }
    return 0;
}
