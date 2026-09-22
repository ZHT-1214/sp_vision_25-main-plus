#include "../config.hpp"
#include "ascii_banner.hpp"
#include "backward-cpp/backward.hpp"
#include "tasks/base/packet_typedef_receive.hpp"
#include "tasks/base/packet_typedef_send.hpp"
#include "utils/drivers/serial_driver.hpp"
#include "utils/runtime_tf.hpp"
#include "utils/signal_guard.hpp"
#include <utility>
namespace backward {
static backward::SignalHandling sh;
}
using namespace awakening;

enum class SimpleFrame : int { ODOM, GIMBAL_ODOM, GIMBAL, CAMERA, CAMERA_CV, SHOOT, N };

using SimpleRobotTF = utils::tf::RobotTF<SimpleFrame, static_cast<size_t>(SimpleFrame::N), false>;

struct SerialTag {};
using SerialIO = IOPair<SerialTag, std::vector<uint8_t>>;

int main(int argc, char** argv) {
    auto start_tp = Clock::now();
    print_banner();
    auto& signal = utils::SignalGuard::instance();
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
    SerialDriver serial(config["serial"], s);
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
    VisionDebugCtx dbg;
    s.register_task<SerialIO>("receive_serial", [&](SerialIO::second_type&& data) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        auto now = Clock::now();

        if (auto robo_opt = ReceiveRobotData::create(data); robo_opt.has_value()) {
            auto robo = robo_opt.value();
            static uint32_t last_pc = -1, delay = 0, last_bullet_count = 0;
            if (robo.time_stamp_pc != last_pc) {
                last_pc = robo.time_stamp_pc;
                delay =
                    (std::chrono::duration_cast<std::chrono::microseconds>(now - start_tp).count()
                     - robo.time_stamp_pc
                     - (robo.time_stamp_send_micro - robo.time_stamp_receive_micro))
                    / 2.0;
            }

            auto packet_time = now - std::chrono::microseconds(delay);
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

            tf->push(SimpleFrame::ODOM, SimpleFrame::GIMBAL_ODOM, packet_time, ISO3::Identity());
            robo.update_log(delay);
        }
    });
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
        auto [pitch, v_pitch, a_pitch] = sample_sine_motion(yaw_amplitude_deg, yaw_frequency_hz, t);

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
        serial.write(std::move(utils::to_vector(send)));
        dbg.gimbal_cmd.set(cmd);
        auto gimbal_in_gimbal_odom =
            tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ODOM, Clock::now());
        auto rpy = utils::matrix2rpy<double>(gimbal_in_gimbal_odom.linear());
        auto gimbal_yaw_pitch =
            std::make_pair(angles::to_degrees(rpy[2]), -angles::to_degrees(rpy[1]));
        dbg.gimbal_yaw_pitch.set(gimbal_yaw_pitch);
        static Web web;
        web.write_debug_data(dbg);
    });
    serial.start<SerialTag>("serial");
    s.build();
    s.run();
    utils::SignalGuard::spin(std::chrono::milliseconds(1000));
    s.stop();
    return 0;
}
