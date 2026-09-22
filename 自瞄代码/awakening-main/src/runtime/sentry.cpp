#include "angles.h"
#include "ascii_banner.hpp"
#include "tasks/auto_aim/armor_omni.hpp"
#include "tasks/auto_buff/rune_control/rune_aimer.hpp"
#include "tasks/auto_buff/rune_detect/rune_detector.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/rune_track/rune_tracker.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/wheel_odometry.hpp"
#include "tasks/sentry_brain/rmuc_2026/mode_factory.hpp"
#include "utils/io/video_save.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#ifdef USE_RERUN
    #include "_rerun/recorder.hpp"
    #include "_rerun/tf.hpp"
#endif
#ifdef USE_ROS2
    #include "_rcl/visual/armor.hpp"
    #include "_rcl/visual/armor_target.hpp"
    #include "sensor_msgs/msg/camera_info.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include <rclcpp/qos.hpp>
#endif
#include "backward-cpp/backward.hpp"
#include "param_deliver.h"
#include "runtime/config.hpp"
#include "tasks/auto_aim/armor_control/very_aimer.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/semaphore_guard.hpp"
#include "utils/signal_guard.hpp"
namespace backward {
static backward::SignalHandling sh;
}
using namespace awakening;
enum class Mode : int { AutoAim = 0, AutoBuff = 1 };
Mode str_to_Mode(std::string str) {
    str = utils::to_upper(str);
    if (str == "AUTO_AIM") {
        return Mode::AutoAim;
    } else if (str == "AUTO_BUFF") {
        return Mode::AutoBuff;
    } else {
        throw std::invalid_argument("Invalid mode string");
    }
}
enum class SentryFrame : int {
    ODOM,
    GIMBAL_ODOM,
    GIMBAL,
    CAMERA,
    CAMERA_CV,
    SHOOT,
    BIG_YAW,
    OMNI_0,
    OMNI_0_CV,
    OMNI_1,
    OMNI_1_CV,
    N
};

using SimpleRobotTF = utils::tf::RobotTF<SentryFrame, static_cast<size_t>(SentryFrame::N), false>;

std::string SentryFrame_to_str(int frame) {
    constexpr const char* details[] = { "odom",      "gimbal_odom", "gimbal",   "camera",
                                        "camera_cv", "shoot",       "big_yaw",  "omni_0",
                                        "omni_0_cv", "omni_1",      "omni_1_cv" };
    return std::string(details[frame]);
}
std::string SentryFrame_to_str(SentryFrame frame) {
    return SentryFrame_to_str(std::to_underlying(frame));
}
struct CameraTag {};
struct SerialTag {};
struct DetectTag {};
struct FrameTag {};

using CameraIO = IOPair<CameraTag, ImageFrame>;
using SerialIO = IOPair<SerialTag, std::vector<uint8_t>>;
using CommonFrameIo = IOPair<FrameTag, CommonFrame>;
struct Detection {
    std::optional<std::vector<auto_aim::Armors>> armors;
    std::optional<std::vector<auto_buff::RuneDetection>> rune;
    std::optional<TimePoint> detect_start;
};
using DetIo = IOPair<DetectTag, Detection>;
struct LogCtx {
    int camera_count = 0;
    int detect_count = 0;
    int track_count = 0;
    int solve_count = 0;
    int serial_count = 0;
    int found_count = 0;
    double latency_ms_total = 0.0;
    int omni_count = 0;
    double omni_latency_ms_total = 0.0;
    void reset() {
        *this = {};
    }
};
struct AutoExposureCfg {
    double target_brightness = 0.0;
    double step_gain = 0.0;
    double decay_step = 0.0;
    double tolerance = 0.0;
    double exposure_min = 0.0;
    double exposure_max = 0.0;
    double control_interval_ms = 0.0;

    explicit AutoExposureCfg(const YAML::Node& c):
        target_brightness(c["target_brightness"].as<double>()),
        step_gain(c["step_gain"].as<double>()),
        decay_step(c["decay_step"].as<double>()),
        tolerance(c["tolerance"].as<double>()),
        exposure_min(c["exposure_min"].as<double>()),
        exposure_max(c["exposure_max"].as<double>()),
        control_interval_ms(c["control_interval_ms"].as<double>()) {}
};
bool is_web_running() {
    static std::atomic<bool> cached { true };
    utils::dt_once(
        [&]() {
            const int ret = std::system("pgrep -x wust_vision_web > /dev/null 2>&1");
            cached = (ret == 0);
        },
        std::chrono::duration<double>(1.0)
    );
    bool running = cached.load();
#ifdef USE_RERUN
    running = running || rerun_visual::Recorder::instance().enabled();
#endif
    return running;
}
static constexpr auto RECORD_FOLDER_PATH_ARR = utils::concat(ROOT_DIR, "/record/auto_aim");
static constexpr std::string_view RECORD_FOLDER_PATH(RECORD_FOLDER_PATH_ARR.data());

int main(int argc, char** argv) {
    auto start_tp = Clock::now();
    print_banner();
    auto& signal = utils::SignalGuard::instance();
    logger::init(spdlog::level::trace);
    bool debug = false;
    std::string config_path;
    std::string robot_name;
    auto first_arg = utils::get_arg(1, argc, argv);
    if (first_arg) {
        robot_name = first_arg.value();
        config_path = get_robot_config_path(robot_name).value_or(robot_name);
    } else {
        return 1;
    }
    auto second_arg = utils::get_arg(2, argc, argv);
    if (second_arg) {
        debug = second_arg.value() == "true";
    }
    auto config = YAML::LoadFile(config_path);

    Scheduler s;
    std::atomic<Mode> mode = str_to_Mode(config["mode"].as<std::string>());
    std::atomic<EnemyColor> enemy_color =
        enemy_color_from_string(config["enemy_color"].as<std::string>());
    std::atomic<double> bullet_speed = config["bullet_speed"].as<double>();
#ifdef USE_ROS2
    rcl::RclcppNode rcl_node("auto_aim");
    rcl::TF rcl_tf(rcl_node);
#endif

    std::shared_ptr<SerialDriver> serial;
    if (config["serial"]["enable"].as<bool>()) {
        serial = std::make_shared<SerialDriver>(config["serial"], s);
    }

    auto camera_config = config["camera"];
    std::unique_ptr<Camera> camera;
    utils::SignalGuard::add_callback([&]() {
        if (camera) {
            camera->stop();
        }
    });

    camera = create_camera(camera_config, s, "hik");
    camera->init();
    if (!camera->is_running()) {
        return 0;
    }
    std::unique_ptr<VideoSaver> video_saver;
    if (config["record"]["enable"].as<bool>()) {
        video_saver = std::make_unique<VideoSaver>(
            VideoSaver::generate_record_filename(RECORD_FOLDER_PATH.data()),
            VideoSaver::Mode::NonBlocking
        );
    }
    CameraInfo camera_info;
    camera_info.load(camera_config["camera_info"]);
    auto_aim::ArmorDetector armor_detector(config["armor_detector"]);
    auto_aim::ArmorTracker armor_tracker(config["armor_tracker"]);
    armor_tracker.set_sentry(true);
    auto_aim::AutoAimFsmController auto_aim_fsm_controller(config["auto_aim_fsm"]);
    auto_aim::ArmorOmni armor_omni(config["armor_omni"]);
    auto_aim::VeryAimer very_aimer(config["very_aimer"]);
    auto_buff::RuneDetector rune_detector(config["rune_detector"]);
    auto_buff::RuneTracker rune_tracker(config["rune_tracker"]);
    auto_buff::RuneAimer rune_aimer(config["rune_aimer"]);
    utils::OrderedQueue<auto_aim::Armors> armors_queue;
    utils::OrderedQueue<auto_buff::RuneDetection> rune_detections_queue;
    utils::SWMR<auto_aim::ArmorTarget> armor_target;
    utils::SWMR<auto_aim::ArmorTarget> omni_armor_target;
    utils::SWMR<auto_buff::RuneTarget> rune_target;
    auto brain = sentry_brain::create_brain_mode(rcl_node, rcl_tf, config["brain"], serial);
    armor_omni.emplace_one(
        config["armor_omni"]["camera0"],
        std::to_underlying(SentryFrame::OMNI_0),
        std::to_underlying(SentryFrame::OMNI_0_CV)
    );
    armor_omni.emplace_one(
        config["armor_omni"]["camera1"],
        std::to_underlying(SentryFrame::OMNI_1),
        std::to_underlying(SentryFrame::OMNI_1_CV)
    );
    BulletPickUp bullet_pick_up(config["bullet_pick_up"]);
    LogCtx log_ctx;
    std::optional<VisionDebugCtx> dbg;
    if (debug) {
        dbg.emplace();
        dbg->camera_info_ = camera_info;
    }
    WheelOdometry wheel_odometry(config["wheel_odometry"], Clock::now());
    auto tf = SimpleRobotTF::create();
    {
        for (auto [from, to]: std::array {
                 std::pair { SentryFrame::ODOM, SentryFrame::GIMBAL_ODOM },
                 std::pair { SentryFrame::GIMBAL_ODOM, SentryFrame::GIMBAL },
                 std::pair { SentryFrame::GIMBAL, SentryFrame::CAMERA },
                 std::pair { SentryFrame::GIMBAL, SentryFrame::SHOOT },
                 std::pair { SentryFrame::CAMERA, SentryFrame::CAMERA_CV },
                 std::pair { SentryFrame::GIMBAL_ODOM, SentryFrame::BIG_YAW },
                 std::pair { SentryFrame::BIG_YAW, SentryFrame::OMNI_0 },
                 std::pair { SentryFrame::OMNI_0, SentryFrame::OMNI_0_CV },
                 std::pair { SentryFrame::BIG_YAW, SentryFrame::OMNI_1 },
                 std::pair { SentryFrame::OMNI_1, SentryFrame::OMNI_1_CV },
             })
        {
            tf->add_edge(from, to);
        }
        ISO3 cv_in_camera = ISO3::Identity();
        cv_in_camera.linear() = R_CV2PHYSICS;
        tf->push(SentryFrame::CAMERA, SentryFrame::CAMERA_CV, Clock::now(), cv_in_camera);
        tf->push(
            SentryFrame::GIMBAL,
            SentryFrame::CAMERA,
            Clock::now(),
            utils::load_isometry3(config["tf"]["camera_in_gimbal"])
        );
        tf->push(
            SentryFrame::GIMBAL,
            SentryFrame::SHOOT,
            Clock::now(),
            utils::load_isometry3(config["tf"]["shoot_in_gimbal"])
        );
        tf->push(
            SentryFrame::BIG_YAW,
            SentryFrame::OMNI_0,
            Clock::now(),
            utils::load_isometry3(config["tf"]["omin_0_in_big_yaw"])
        );
        tf->push(
            SentryFrame::BIG_YAW,
            SentryFrame::OMNI_1,
            Clock::now(),
            utils::load_isometry3(config["tf"]["omin_1_in_big_yaw"])
        );
        tf->push(SentryFrame::OMNI_0, SentryFrame::OMNI_0_CV, Clock::now(), cv_in_camera);
        tf->push(SentryFrame::OMNI_1, SentryFrame::OMNI_1_CV, Clock::now(), cv_in_camera);
    }
    if (video_saver) {
        s.register_task<CameraIO>("save_video", [&](CameraIO::second_type&& f) {
            if (!f.src_img.empty()) {
                video_saver->write_frame(f.src_img);
            }
            return std::make_tuple(std::optional<CameraIO::second_type>(std::nullopt));
        });
    }
    auto serial_send_to_image_microseconds = config["serial_send_to_image_microseconds"].as<int>();

    s.register_task<CameraIO, CommonFrameIo>("push_common_frame", [&](CameraIO::second_type&& f) {
        static int current_id = 0;
        if (f.src_img.empty()) {
            return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::nullopt));
        }
        log_ctx.camera_count++;

        CommonFrame frame {
            .img_frame = std::move(f),
            .id = current_id++,
            .frame_id = std::to_underlying(SentryFrame::CAMERA_CV),

        };

        return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::move(frame)));
    });
    if (serial) {
        s.register_task<SerialIO>("receive_serial", [&](SerialIO::second_type&& data) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            auto now = Clock::now();

            log_ctx.serial_count++;
            if (auto robo_opt = ReceiveRobotData::create(data); robo_opt.has_value()) {
                auto robo = robo_opt.value();
                static uint32_t last_pc = -1, delay = 0, last_bullet_count = 0;
                if (robo.time_stamp_pc != last_pc) {
                    last_pc = robo.time_stamp_pc;
                    delay = (std::chrono::duration_cast<std::chrono::microseconds>(now - start_tp)
                                 .count()
                             - robo.time_stamp_pc
                             - (robo.time_stamp_send_micro - robo.time_stamp_receive_micro))
                        / 2.0;
                }
                auto packet_time =
                    now - std::chrono::microseconds(serial_send_to_image_microseconds);
                ISO3 gimbal_in_gimbal_odom = ISO3::Identity();
                gimbal_in_gimbal_odom.linear() = utils::rpy2matrix(Vec3(
                    angles::from_degrees(robo.roll),
                    angles::from_degrees(robo.pitch),
                    angles::from_degrees(robo.yaw)
                ));
                tf->push(
                    SentryFrame::GIMBAL_ODOM,
                    SentryFrame::GIMBAL,
                    packet_time,
                    gimbal_in_gimbal_odom
                );
                auto gimbal_odom_in_odom = ISO3::Identity();
                gimbal_odom_in_odom.translation() = Vec3(robo.v_x, robo.v_y, robo.v_z);
                tf->push(
                    SentryFrame::ODOM,
                    SentryFrame::GIMBAL_ODOM,
                    packet_time,
                    gimbal_odom_in_odom
                );
                enemy_color = EnemyColor(robo.detect_color);
                robo.update_log(delay);
                if (robo.bullet_count > last_bullet_count) {
                    bullet_pick_up.push_back(Bullet {
                        .fire_time = Clock::now(),
                        .fire_time_shoot_in_odom =
                            tf->pose_a_in_b(SentryFrame::SHOOT, SentryFrame::ODOM, Clock::now()),
                        .speed_in_odom = bullet_speed,
                    });
                }
                last_bullet_count = robo.bullet_count;
            }
            if (auto joint_opt = SentryJointState::create(data); joint_opt.has_value()) {
                auto joint = joint_opt.value();
                auto gimbal_2_gimbal_odom =
                    tf->pose_a_in_b(SentryFrame::GIMBAL, SentryFrame::GIMBAL_ODOM, Clock::now());
                auto rpy = utils::matrix2rpy<double>(gimbal_2_gimbal_odom.linear());
                rpy[2] = angles::from_degrees(joint.big_yaw_in_world);
                rpy[1] = 0.0;
                ISO3 big_yaw_2_gimbal_odom = ISO3::Identity();
                big_yaw_2_gimbal_odom.translation() = Vec3(0, 0, 0.0);
                big_yaw_2_gimbal_odom.linear() = utils::rpy2matrix(rpy);
                tf->push(
                    SentryFrame::GIMBAL_ODOM,
                    SentryFrame::BIG_YAW,
                    Clock::now(),
                    big_yaw_2_gimbal_odom
                );
            }
            auto referee_opt = SentryRefereeReceive::create(data);
            if (referee_opt && brain) {
                brain->update_gobal_state(referee_opt.value());
            }
        });
    }
    if (camera) {
        s.register_task<CommonFrameIo>("auto_exposure", [&](CommonFrameIo::second_type&& frame) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            static std::optional<AutoExposureCfg> auto_exposure_cfg =
                config["auto_exposure"]["enable"].as<bool>()
                ? std::optional<AutoExposureCfg>(std::in_place, config["auto_exposure"])
                : std::nullopt;
            if (auto_exposure_cfg) {
                auto& cfg = auto_exposure_cfg.value();
                utils::dt_once(
                    [&]() {
                        cv::Mat gray;
                        cv::cvtColor(frame.img_frame.src_img, gray, cv::COLOR_BGR2GRAY);
                        double exposure_time = camera->get_ExposureTime();
                        static double last_exposure_time = 0.0;
                        const double diff = cv::mean(gray)[0] - cfg.target_brightness;
                        if (std::fabs(diff) > cfg.tolerance && exposure_time > 0.0) {
                            exposure_time -= diff * cfg.step_gain;
                        } else {
                            exposure_time -= cfg.decay_step;
                        }
                        exposure_time =
                            std::clamp(exposure_time, cfg.exposure_min, cfg.exposure_max);
                        if (std::abs(exposure_time - last_exposure_time) > 10) {
                            camera->set_ExposureTime(exposure_time);
                            last_exposure_time = exposure_time;
                        }
                    },
                    std::chrono::milliseconds((int)cfg.control_interval_ms)
                );
            }
        });
    }

    s.register_task<CommonFrameIo, DetIo>("detector", [&](CommonFrameIo::second_type&& frame) {
        Detection r;
        cv::Rect net_focus =
            cv::Rect(0, 0, frame.img_frame.src_img.cols, frame.img_frame.src_img.rows);
        if (mode == Mode::AutoAim) {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["armor_detector"]["max_infer_num"].as<int>()
                );
            }
            std::optional<cv::Rect> detect_light = std::nullopt;
            auto target = armor_target.read();
            if (target.need_focus()) {
                auto camera_cv_in_old = tf->pose_a_in_b(
                    SentryFrame(frame.frame_id),
                    SentryFrame(target.get_target_state().frame_id),
                    frame.img_frame.timestamp
                );
                net_focus = target.get_net_focus_roi(
                    frame.img_frame.timestamp,
                    camera_cv_in_old,
                    camera_info,
                    frame.img_frame.src_img.size(),
                    armor_detector.get_net_wh_ratio()
                );
                if (target.need_detect_lights()) {
                    detect_light = target.expanded( // 送给传统越小越好
                        frame.img_frame.timestamp,
                        camera_cv_in_old,
                        camera_info,
                        frame.img_frame.src_img.size()
                    );
                    detect_light = utils::expand_and_clip_rect(
                        detect_light.value(),
                        1.6,
                        frame.img_frame.src_img.size()
                    );
                }
            }

            auto_aim::Armors armors { .timestamp = frame.img_frame.timestamp,
                                      .id = frame.id,
                                      .frame_id = frame.frame_id };
            {
                bool got = detector_sem->try_acquire();
                utils::SemaphoreGuard guard(*detector_sem, got); //并发控制
                if (got) {
                    auto start = Clock::now();
                    r.detect_start = start;
                    auto [ls, as] = armor_detector.detect(frame, net_focus, detect_light);
                    armors.armors = as;
                    armors.lights = ls;
                    log_ctx.detect_count++;
                }
            }
            armors_queue.enqueue(armors);
            auto batch_armors = armors_queue.dequeue_batch(); // 根据id有序输出
            r.armors = batch_armors;
        } else {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["rune_detector"]["max_infer_num"].as<int>()
                );
            }
            auto target = rune_target.read();
            if (target.need_focus()) {
                auto camera_cv_in_old = tf->pose_a_in_b(
                    SentryFrame(frame.frame_id),
                    SentryFrame(target.get_target_state().frame_id),
                    frame.img_frame.timestamp
                );
                net_focus = target.get_net_focus_roi(
                    frame.img_frame.timestamp,
                    camera_cv_in_old,
                    camera_info,
                    frame.img_frame.src_img.size(),
                    armor_detector.get_net_wh_ratio()
                );
            }
            auto_buff::RuneDetection rune_detection;
            {
                bool got = detector_sem->try_acquire();
                utils::SemaphoreGuard guard(*detector_sem, got); //并发控制
                if (got) {
                    auto start = Clock::now();
                    r.detect_start = start;
                    rune_detection = rune_detector.detect(frame, net_focus, enemy_color);
                    log_ctx.detect_count++;
                }
            }
            rune_detection.frame_id = frame.frame_id;
            rune_detection.id = frame.id;
            rune_detection.timestamp = frame.img_frame.timestamp;
            rune_detections_queue.enqueue(rune_detection);
            auto batch_runes = rune_detections_queue.dequeue_batch();
            r.rune = batch_runes;
        }

        if (dbg && is_web_running()) {
            dbg->expanded.set(net_focus);
            dbg->img_frame.set(std::move(frame.img_frame));
        }

        return std::make_tuple(std::optional<DetIo::second_type>(std::move(r)));
    });

    s.register_task<DetIo>("tracker", [&](DetIo::second_type&& io) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        if (io.armors) {
            for (const auto& armors_raw: io.armors.value()) {
                auto armors = armors_raw;
                auto is_ally = [&](const auto& obj) {
                    return (enemy_color == EnemyColor::BLUE
                            && obj.color == auto_aim::ArmorColor::RED)
                        || (enemy_color == EnemyColor::RED
                            && obj.color == auto_aim::ArmorColor::BLUE);
                };
                armors.armors.erase(
                    std::remove_if(armors.armors.begin(), armors.armors.end(), is_ally),
                    armors.armors.end()
                );
                armors.lights.erase(
                    std::remove_if(armors.lights.begin(), armors.lights.end(), is_ally),
                    armors.lights.end()
                );
                auto camera_cv_in_odom = tf->pose_a_in_b(
                    SentryFrame(armors.frame_id),
                    SentryFrame::ODOM,
                    armors.timestamp
                ); //转到odom
                armors.frame_id = std::to_underlying(SentryFrame::ODOM);
                auto __armor_target =
                    armor_tracker.track(armors, camera_info, camera_cv_in_odom, armors.frame_id);
                auto_aim_fsm_controller.update(
                    __armor_target.get_target_state().vyaw(),
                    __armor_target.jumped,
                    __armor_target.get_target_state().timestamp
                );
                armor_target.write(__armor_target);
                if (io.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - io.detect_start.value()
                    )
                                          .count();
                    log_ctx.latency_ms_total += latency_ms;
                }

                log_ctx.found_count += armor_tracker.get_count();
                armor_tracker.reset_count();
                if (dbg) {
                    dbg->armors.set(armors);
#ifdef USE_ROS2
                    rcl::pub_armor_marker(rcl_node, SentryFrame_to_str(armors.frame_id), armors);
                    rcl::pub_armor_target_marker(
                        rcl_node,
                        SentryFrame_to_str(__armor_target.get_target_state().frame_id),
                        __armor_target
                    );
#endif
                }

                log_ctx.track_count++;
            }
        } else if (io.rune) {
            for (auto& rune: io.rune.value()) {
                auto camera_cv_in_odom = tf->pose_a_in_b(
                    SentryFrame(rune.frame_id),
                    SentryFrame::ODOM,
                    rune.timestamp
                ); //转到odom
                rune.frame_id = std::to_underlying(SentryFrame::ODOM);

                auto __rune_target =
                    rune_tracker.track(rune, camera_info, camera_cv_in_odom, rune.frame_id);

                rune_target.write(__rune_target);
                if (io.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - io.detect_start.value()
                    )
                                          .count();
                    log_ctx.latency_ms_total += latency_ms;
                }
                log_ctx.found_count += rune_tracker.get_count();
                rune_tracker.reset_count();

                if (dbg) {
                    dbg->rune.set(rune);
                }
                log_ctx.track_count++;
            }
        }
    });
    s.add_rate_source<>("solver", 1000.0, [&]() {
        log_ctx.solve_count++;
        bool is_omni = false;
        auto a_target = armor_target.read(); //需要转为相对gimbal_odom
        if (!a_target.check()) {
            a_target = omni_armor_target.read();
            is_omni = true;
        }
        int old_this_id = a_target.this_id;
        auto gimbal_odom_state_in_odom = wheel_odometry.state;
        gimbal_odom_state_in_odom.predict(Clock::now());
        a_target.set_target_state([&](auto& s) {
            namespace idx = auto_aim::armor_point_motion_model::idx;
            s.frame_id = std::to_underlying(SentryFrame::GIMBAL_ODOM);
            const auto pos = gimbal_odom_state_in_odom.pos();
            const auto vel = gimbal_odom_state_in_odom.vel();
            s.x[idx::CX] -= pos.x();
            s.x[idx::CY] -= pos.y();
            s.x[idx::CZ] -= pos.z();
            s.x[idx::VCX] -= vel.x();
            s.x[idx::VCY] -= vel.y();
            s.x[idx::VCZ] -= vel.z();
        });
        a_target.this_id = old_this_id;
        auto r_target = rune_target.read();
        old_this_id = r_target.this_id;
        r_target.set_target_state([&](auto& s) {
            namespace idx = auto_buff::motion_model::idx;
            s.frame_id = std::to_underlying(SentryFrame::GIMBAL_ODOM);
            const auto pos = gimbal_odom_state_in_odom.pos();
            const auto vel = gimbal_odom_state_in_odom.vel();
            s.x[idx::CX] -= pos.x();
            s.x[idx::CY] -= pos.y();
            s.x[idx::CZ] -= pos.z();
        });
        r_target.this_id = old_this_id;

        GimbalCmd cmd {
            .appear = false,
        };
        auto shoot_in_gimbal_odom =
            tf->pose_a_in_b(SentryFrame::SHOOT, SentryFrame::GIMBAL_ODOM, Clock::now());
        auto gimbal_in_gimbal_odom =
            tf->pose_a_in_b(SentryFrame::GIMBAL, SentryFrame::GIMBAL_ODOM, Clock::now());
        if (a_target.check()) {
            cmd = very_aimer.very_aim(
                a_target,
                bullet_speed,
                auto_aim_fsm_controller.get_state(),
                shoot_in_gimbal_odom,
                gimbal_in_gimbal_odom
            );

        } else if (r_target.check()) {
            cmd =
                rune_aimer.aim(r_target, bullet_speed, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
        }

        if (serial) {
            SendRobotCmdData send;
            send.cmd_ID = SendRobotCmdData::ID;
            send.time_stamp =
                std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp)
                    .count();
            send.appear = cmd.appear, send.detect_color = std::to_underlying(enemy_color.load());
            send.yaw = cmd.yaw, send.pitch = cmd.pitch, send.v_yaw = cmd.v_yaw;
            send.target_yaw = cmd.target_yaw, send.target_pitch = cmd.target_pitch;
            send.v_pitch = cmd.v_pitch, send.a_yaw = cmd.a_yaw, send.a_pitch = cmd.a_pitch;
            send.enable_yaw_diff = cmd.enable_yaw_diff;
            send.enable_pitch_diff = cmd.enable_pitch_diff;
            serial->write(std::move(utils::to_vector(send)));
        }
        auto old_in_camera_cv = tf->pose_a_in_b(
            SentryFrame(cmd.aim_point.frame_id),
            SentryFrame::CAMERA_CV,
            cmd.timestamp
        );
        cmd.aim_point.transform(old_in_camera_cv, std::to_underlying(SentryFrame::CAMERA_CV));
        if (dbg && is_web_running()) {
            dbg->gimbal_cmd.set(cmd);
        }
    });
    auto armor_omni_task = s.register_source<>("armor_omni");
    s.add_rate_source<>("armor_omni_tragger", armor_omni.fps_, [&]() {
        auto main_target = armor_target.read();
        if (main_target.check()) {
            return;
        }
        auto_aim::ArmorOmni::One& one = armor_omni.get_next();
        auto img_frame = one.camera->read();
        if (img_frame.src_img.empty()) {
            AWAKENING_WARN("Failed to read image from camera.");
            return;
        }

        CommonFrame common_frame;
        common_frame.img_frame = std::move(img_frame);
        common_frame.frame_id = one.cv_frame_id;
        common_frame.id = one.order_id++;

        s.runtime_push_source<>(armor_omni_task, [&, f = std::move(common_frame)]() {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["armor_omni"]["max_infer_num"].as<int>()
                );
            }
            {
                auto_aim::Armors armors { .timestamp = f.img_frame.timestamp,
                                          .id = f.id,
                                          .frame_id = f.frame_id };
                {
                    bool got = detector_sem->try_acquire();
                    utils::SemaphoreGuard guard(*detector_sem, got);
                    if (got) {
                        auto [tmp_lights, tmp_armors] = armor_omni.detector_->detect(
                            f,
                            cv::Rect(0, 0, f.img_frame.src_img.cols, f.img_frame.src_img.rows)
                        );
                        armors.lights = tmp_lights;
                        armors.armors = tmp_armors;
                        for (auto& a: tmp_armors) {
                            if ((enemy_color == EnemyColor::BLUE
                                 && a.color == auto_aim::ArmorColor::RED)
                                || (enemy_color == EnemyColor::RED
                                    && a.color == auto_aim::ArmorColor::BLUE))
                            {
                                continue;
                            }
                            armors.armors.push_back(a);
                        }
                    }
                }

                one.armors_queue->enqueue(armors);
            }

            auto batch_armors = one.armors_queue->dequeue_batch();
            {
                static std::mutex mutex;
                std::lock_guard<std::mutex> lock(mutex);
                for (auto& _armors: batch_armors) {
                    auto camera_cv_in_odom = tf->pose_a_in_b(
                        SentryFrame(_armors.frame_id),
                        SentryFrame::ODOM,
                        _armors.timestamp
                    );
                    _armors.frame_id = std::to_underlying(SentryFrame::ODOM);

                    one.target =
                        one.tracker
                            ->track(_armors, camera_info, camera_cv_in_odom, _armors.frame_id);
                    if (!one.target.check()) {
                        one.target.update_count = 0;
                    }
                    one.total_score = one.target.update_count;
                    auto best_target = armor_omni.update();
                    omni_armor_target.write(best_target);
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - _armors.timestamp
                    )
                                          .count();
                    log_ctx.omni_latency_ms_total += latency_ms;
                    log_ctx.omni_count++;
                    if (dbg) {
#ifdef USE_ROS2
                        rcl::pub_armor_target_marker(
                            rcl_node,
                            SentryFrame_to_str(best_target.get_target_state().frame_id),
                            best_target
                        );
#endif
                    }
                }
            }
        });
    });
    s.add_rate_source<>("logger", 1.0, [&]() {
        double avg_latency_ms = log_ctx.latency_ms_total / log_ctx.track_count;
        double omni_avg_latency_ms = log_ctx.omni_latency_ms_total / log_ctx.omni_count;
        AWAKENING_INFO(
            "detect: {} track: {} found: {} solve: {} serial: {} camera: {} avg_latency: {:.3} ms omni: {} avg_latency: {:.3} ms ",
            log_ctx.detect_count,
            log_ctx.track_count,
            log_ctx.found_count,
            log_ctx.solve_count,
            log_ctx.serial_count,
            log_ctx.camera_count,
            avg_latency_ms,
            log_ctx.omni_count,
            omni_avg_latency_ms
        );
        if (dbg) {
            dbg->avg_latency_ms.set(avg_latency_ms);
        }
        log_ctx.reset();
    });
    if (dbg) {
        s.add_rate_source<>("debug", 45.0, [&]() {
            if (!is_web_running()) {
                return;
            }
            static Web web;
            dbg->type =
                (mode == Mode::AutoAim) ? VisionDebugCtx::AUTO_AIM : VisionDebugCtx::AUTO_BUFF;
            auto __armor_target = armor_target.read();
            auto __rune_target = rune_target.read();
            __armor_target.write_log();
            __rune_target.write_log();
            wheel_odometry.write_log();
            auto img_now = dbg->img_frame.get().timestamp;
            dbg->armor_target.set(__armor_target);
            dbg->rune_target.set(__rune_target);
            dbg->auto_aim_fsm_state.set(auto_aim_fsm_controller.get_state());
            auto gimbal_in_gimbal_odom =
                tf->pose_a_in_b(SentryFrame::GIMBAL, SentryFrame::GIMBAL_ODOM, Clock::now());
            auto rpy = utils::matrix2rpy<double>(gimbal_in_gimbal_odom.linear());
            auto gimbal_yaw_pitch =
                std::make_pair(angles::to_degrees(rpy[2]), -angles::to_degrees(rpy[1]));
            dbg->gimbal_yaw_pitch.set(gimbal_yaw_pitch);
            web.write_debug_data(dbg.value());
            bullet_pick_up.update(
                Clock::now(),
                dbg->gimbal_cmd.get().appear ? dbg->gimbal_cmd.get().fly_time : 0.4
            );
            auto bullet_poss =
                bullet_pick_up.get_bullet_positions(img_now, very_aimer.get_yaw_pitch_offset());
            auto odom_in_camera_cv =
                tf->pose_a_in_b(SentryFrame::ODOM, SentryFrame::CAMERA_CV, img_now);
            for (auto& pos: bullet_poss) {
                pos = odom_in_camera_cv * pos;
            }
            dbg->odom_in_camera_cv.set(odom_in_camera_cv);
            dbg->bullet_positions.set(bullet_poss);
            auto img = dbg->img_frame.get();
            auto debug_img = img.src_img.clone();
            if (img.format == PixelFormat::RGB) {
                cv::cvtColor(debug_img, debug_img, cv::COLOR_RGB2BGR);
            }
            if (!debug_img.empty()) {
                web.draw(debug_img, dbg.value());
                web.write_shm(debug_img);
            }
        });
#ifdef USE_ROS2
        if (debug) {
            s.add_rate_source<>("tf_pub", 100.0, [&]() {
                rcl_tf.pub_robot_tf(*tf, [](SentryFrame frame) {
                    return SentryFrame_to_str(frame);
                });
            });
        }
#endif
#ifdef USE_RERUN
        if (debug) {
            s.add_rate_source<>("rerun_tf", 15.0, [&]() {
                rerun_visual::log_robot_tf(*tf, [](SentryFrame frame) {
                    return SentryFrame_to_str(frame);
                });
            });
        }
#endif
    }
    auto cmd_sub = rcl_node.make_sub<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            SendNavCmdData send;

            send.cmd_ID = SendNavCmdData::ID;
            send.vx = msg->linear.x;
            send.vy = msg->linear.y;
            send.wz = msg->angular.z;
            send.turtle_state = (msg->linear.z > 0) ? true : false;
            if (serial) {
                serial->write(utils::to_vector(send));
            }
        }
    );

    rcl_node.push_sub(cmd_sub);

    if (camera) {
        camera->start<CameraTag>("hik");
    }

    if (serial) {
        serial->start<SerialTag>("serial");
    }

    if (brain) {
        brain->start();
    }
    s.build();
    s.run();

#ifdef USE_ROS2
    std::thread([&]() { rcl_node.spin(); }).detach();
#endif
    utils::SignalGuard::spin(std::chrono::milliseconds(1000));
    s.stop();
#ifdef USE_ROS2
    rcl_node.shutdown();
#endif
    for (int i = 0; i < 10; ++i) {
        AWAKENING_CRITICAL("改了东西记得同步其他有关的exe的src");
    }
    return 0;
}
