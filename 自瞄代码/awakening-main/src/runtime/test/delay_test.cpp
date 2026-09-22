#include "../config.hpp"
#include "ascii_banner.hpp"
#include "backward-cpp/backward.hpp"
#include "tasks/base/common.hpp"
#include "tasks/base/packet_typedef_receive.hpp"
#include "tasks/base/packet_typedef_send.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/drivers/serial_driver.hpp"
#include "utils/runtime_tf.hpp"
#include "utils/signal_guard.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>
using namespace awakening;
namespace backward {
static backward::SignalHandling sh;
}
enum class SimpleFrame : int { ODOM, GIMBAL_ODOM, GIMBAL, CAMERA, CAMERA_CV, SHOOT, N };

using SimpleRobotTF = utils::tf::RobotTF<SimpleFrame, static_cast<size_t>(SimpleFrame::N), false>;

struct CameraTag {};
struct SerialTag {};

template<typename Duration>
static double seconds(const Duration& duration) {
    return std::chrono::duration<double>(duration).count();
}
using CameraIO = IOPair<CameraTag, ImageFrame>;
using SerialIO = IOPair<SerialTag, std::vector<uint8_t>>;

struct Chessboard {
    Vec3 pos_in_camera_cv;
    TimePoint time_stamp;
};
struct CameraCVPose {
    ISO3 pose;
    TimePoint base_time_stamp;
};
int main(int argc, char** argv) {
    auto start_tp = Clock::now();
    print_banner();
    utils::SignalGuard::instance();
    logger::init(spdlog::level::trace);

    std::string config_path;
    std::string robot_name;
    auto first_arg = utils::get_arg(1, argc, argv);
    if (first_arg) {
        robot_name = first_arg.value();
        config_path = get_robot_config_path(robot_name).value_or(robot_name);
    } else {
        return 1;
    }
    auto config = YAML::LoadFile(config_path);
    Scheduler s;

    std::unique_ptr<SerialDriver> serial;
    if (config["serial"]["enable"].as<bool>()) {
        serial = std::make_unique<SerialDriver>(config["serial"], s);
    } else {
        std::cerr << "delay_test requires serial.enable=true to collect gimbal poses" << std::endl;
        return 1;
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

    CameraInfo camera_info;
    camera_info.load(camera_config["camera_info"]);

    auto tf = SimpleRobotTF::create();
    {
        for (auto [from, to]: std::array {
                 std::pair { SimpleFrame::ODOM, SimpleFrame::GIMBAL_ODOM },
                 std::pair { SimpleFrame::GIMBAL_ODOM, SimpleFrame::GIMBAL },
                 std::pair { SimpleFrame::GIMBAL, SimpleFrame::CAMERA },
                 std::pair { SimpleFrame::GIMBAL, SimpleFrame::SHOOT },
                 std::pair { SimpleFrame::CAMERA, SimpleFrame::CAMERA_CV },
             })
        {
            tf->add_edge(from, to);
        }
        ISO3 cv_in_camera = ISO3::Identity();
        cv_in_camera.linear() = R_CV2PHYSICS;
        tf->push(SimpleFrame::CAMERA, SimpleFrame::CAMERA_CV, Clock::now(), cv_in_camera);
        tf->push(
            SimpleFrame::GIMBAL,
            SimpleFrame::CAMERA,
            Clock::now(),
            utils::load_isometry3(config["tf"]["camera_in_gimbal"])
        );
        tf->push(
            SimpleFrame::GIMBAL,
            SimpleFrame::SHOOT,
            Clock::now(),
            utils::load_isometry3(config["tf"]["shoot_in_gimbal"])
        );
    }
    auto serial_send_to_image_microseconds = config["serial_send_to_image_microseconds"].as<int>();
    std::vector<Chessboard> chessboards;
    std::vector<CameraCVPose> pose_buffer;
    std::mutex chessboards_mutex;
    std::mutex pose_buffer_mutex;
    s.register_task<CameraIO>("push_common_frame", [&](CameraIO::second_type&& f) {
        if (f.src_img.empty()) {
            return;
        }
        auto& img = f.src_img;
        std::vector<cv::Point2f> corners_2d;
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        cv::Size pattern_size(11, 8);
        bool success = cv::findChessboardCorners(img, pattern_size, corners_2d, flags);
        if (!success) {
            return;
        }
        cv::Mat gray;
        if (img.channels() == 3)
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
            gray = img;
        cv::cornerSubPix(
            gray,
            corners_2d,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01)
        );
        auto corners_3d = [](const cv::Size& pattern_size, float square_size_m) {
            std::vector<cv::Point3f> pts;
            pts.reserve(pattern_size.width * pattern_size.height);
            for (int i = 0; i < pattern_size.height; i++) {
                for (int j = 0; j < pattern_size.width; j++) {
                    float x = 0.0f;
                    float y = (-j + 0.5f * pattern_size.width) * square_size_m;
                    float z = (-i + 0.5f * pattern_size.height) * square_size_m;
                    pts.push_back({ x, y, z });
                }
            }
            return pts;
        };
        double square_size_m = 0.15;
        cv::Mat rvec, tvec;
        auto corners_3d_ = corners_3d(pattern_size, static_cast<float>(square_size_m));
        success = cv::solvePnP(
            corners_3d_,
            corners_2d,
            camera_info.camera_matrix,
            camera_info.distortion_coefficients,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE
        );
        if (!success) {
            return;
        }
        Chessboard b;
        b.pos_in_camera_cv = Vec3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        b.time_stamp = f.timestamp;
        std::lock_guard<std::mutex> lock(chessboards_mutex);
        chessboards.push_back(b);
    });

    if (serial) {
        s.register_task<SerialIO>("receive_serial", [&](SerialIO::second_type&& data) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            auto now = Clock::now();

            if (auto robo_opt = ReceiveRobotData::create(data); robo_opt.has_value()) {
                auto robo = robo_opt.value();
                auto packet_time =
                    now - std::chrono::microseconds(serial_send_to_image_microseconds);
                ISO3 gimbal_2_gimbal_odom = ISO3::Identity();
                gimbal_2_gimbal_odom.linear() = utils::rpy2matrix(Vec3(
                    angles::from_degrees(robo.roll),
                    angles::from_degrees(robo.pitch),
                    angles::from_degrees(robo.yaw)
                ));
                tf->push(
                    SimpleFrame::GIMBAL_ODOM,
                    SimpleFrame::GIMBAL,
                    packet_time,
                    gimbal_2_gimbal_odom
                );
                tf->push(
                    SimpleFrame::ODOM,
                    SimpleFrame::GIMBAL_ODOM,
                    packet_time,
                    ISO3::Identity()
                );
                CameraCVPose camera_cv_pose;
                camera_cv_pose.pose =
                    tf->pose_a_in_b(SimpleFrame::CAMERA_CV, SimpleFrame::ODOM, packet_time);
                camera_cv_pose.base_time_stamp = packet_time;
                std::lock_guard<std::mutex> pose_lock(pose_buffer_mutex);
                pose_buffer.push_back(camera_cv_pose);
            }
        });
    }

    double yaw_amplitude_deg = 30.0;
    double yaw_frequency_hz = 1;
    double pitch_amplitude_deg = 5.0;
    double pitch_frequency_hz = 1;
    s.add_rate_source<>("solver", 1000.0, [&]() {
        auto now = Clock::now();
        double t = std::chrono::duration<double>(now - start_tp).count(); // 秒
        GimbalCmd cmd;
        auto sample_sine_motion = [](double amplitude, double frequency, double t) {
            const double w = 2.0 * M_PI * frequency;
            const double wt = w * t;

            const double pos = amplitude * std::sin(wt);
            const double vel = amplitude * w * std::cos(wt);
            const double acc = -amplitude * w * w * std::sin(wt);

            return std::make_tuple(pos, vel, acc);
        };
        auto [yaw, v_yaw, a_yaw] = sample_sine_motion(yaw_amplitude_deg, yaw_frequency_hz, t);
        cmd.yaw = yaw;
        cmd.v_yaw = v_yaw;
        cmd.a_yaw = a_yaw;
        auto [pitch, v_pitch, a_pitch] =
            sample_sine_motion(pitch_amplitude_deg, pitch_frequency_hz, t);

        cmd.yaw = yaw;
        cmd.pitch = pitch;
        cmd.v_yaw = v_yaw;
        cmd.v_pitch = v_pitch;
        cmd.a_yaw = a_yaw;
        cmd.a_pitch = a_pitch;
        cmd.appear = true;
        SendRobotCmdData send;
        send.cmd_ID = SendRobotCmdData::ID;
        send.time_stamp =
            std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp).count();
        send.appear = cmd.appear, send.detect_color = 0;
        send.yaw = cmd.yaw, send.pitch = cmd.pitch, send.v_yaw = cmd.v_yaw;
        send.target_yaw = cmd.target_yaw, send.target_pitch = cmd.target_pitch;
        send.v_pitch = cmd.v_pitch, send.a_yaw = cmd.a_yaw, send.a_pitch = cmd.a_pitch;
        send.enable_yaw_diff = cmd.enable_yaw_diff;
        send.enable_pitch_diff = cmd.enable_pitch_diff;
        if (serial) {
            serial->write(std::move(utils::to_vector(send)));
        }
    });
    if (camera) {
        camera->start<CameraTag>("hik");
    }
    if (serial) {
        serial->start<SerialTag>("serial");
    }
    s.build();
    s.run();

    utils::SignalGuard::spin(std::chrono::milliseconds(1000));
    s.stop();

    std::sort(chessboards.begin(), chessboards.end(), [](const Chessboard& a, const Chessboard& b) {
        return a.time_stamp < b.time_stamp;
    });
    std::sort(
        pose_buffer.begin(),
        pose_buffer.end(),
        [](const CameraCVPose& a, const CameraCVPose& b) {
            return a.base_time_stamp < b.base_time_stamp;
        }
    );
    std::cout << "Collected chessboards: " << chessboards.size()
              << ", poses: " << pose_buffer.size() << std::endl;
    if (chessboards.empty() || pose_buffer.size() < 2) {
        std::cerr << "Not enough samples to estimate delay" << std::endl;
        return 1;
    }

    auto search_pose = [&](TimePoint t) -> std::optional<ISO3> {
        auto it = std::lower_bound(
            pose_buffer.begin(),
            pose_buffer.end(),
            t,
            [](const CameraCVPose& p, const TimePoint& t) { return p.base_time_stamp < t; }
        );
        if (it == pose_buffer.begin()) {
            return std::nullopt;
        }

        if (it == pose_buffer.end()) {
            return std::nullopt;
        }
        const auto& p1 = *(it - 1);
        const auto& p2 = *it;

        const double dt = seconds(p2.base_time_stamp - p1.base_time_stamp);
        if (std::abs(dt) < 1e-6)
            return p2.pose;

        double r = std::clamp(seconds(t - p1.base_time_stamp) / dt, 0.0, 1.0);
        Vec3 trans = (1 - r) * p1.pose.translation() + r * p2.pose.translation();
        Quaternion q1(p1.pose.rotation());
        Quaternion q2(p2.pose.rotation());
        q1.normalize();
        q2.normalize();
        if (q1.dot(q2) < 0.0)
            q2.coeffs() *= -1.0;
        Quaternion q = q1.slerp(r, q2).normalized();

        ISO3 T = ISO3::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = trans;
        return T;
    };
    auto cal_error = [&](int delay_us) {
        std::vector<Vec3> positions;
        positions.reserve(chessboards.size());
        for (const auto& chessboard: chessboards) {
            auto img_t = chessboard.time_stamp + std::chrono::microseconds(delay_us);
            auto pose = search_pose(img_t);
            if (pose) {
                positions.push_back(pose.value() * chessboard.pos_in_camera_cv);
            }
        }
        if (positions.size() < 2) {
            return std::numeric_limits<double>::max();
        }

        Vec3 mean = Vec3::Zero();
        for (const auto& pos: positions) {
            mean += pos;
        }
        mean /= static_cast<double>(positions.size());

        double total_error = 0.0;
        for (const auto& pos: positions) {
            total_error += (pos - mean).squaredNorm();
        }
        return std::sqrt(total_error / static_cast<double>(positions.size()));
    };
    int delay_us_start = -10000;
    int delay_us_end = 10000;
    int delay_us_step = 100;
    int best_delay_us = 0;
    double best_error = std::numeric_limits<double>::max();
    for (int delay_us = delay_us_start; delay_us <= delay_us_end; delay_us += delay_us_step) {
        double error = cal_error(delay_us);
        std::cout << "Testing delay: " << delay_us << " us, error: " << error << std::endl;
        if (error < best_error) {
            best_error = error;
            best_delay_us = delay_us;
        }
    }
    if (!std::isfinite(best_error)) {
        std::cerr << "No delay candidate had enough matched samples" << std::endl;
        return 1;
    }
    std::cout << "Best delay: " << best_delay_us << " us, error: " << best_error << std::endl;
    return 0;
}
