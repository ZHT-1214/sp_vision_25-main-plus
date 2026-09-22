#pragma once

#include "daedalus_interface/shm_layout.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/common.hpp"
#include "utils/buffer.hpp"
#include "utils/impl.hpp"
#include "utils/logger.hpp"
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#ifdef USE_RERUN
    #include "_rerun/recorder.hpp"
#endif
namespace awakening {
struct VisionDebugCtx {
    CameraInfo camera_info_; // 基本不变数据，无锁

    utils::Locked<ImageFrame> img_frame;
    utils::Locked<cv::Rect2f> expanded;
    utils::Locked<double> avg_latency_ms;
    utils::Locked<GimbalCmd> gimbal_cmd;
    utils::Locked<std::pair<double, double>> gimbal_yaw_pitch;
    utils::Locked<std::vector<Vec3>> bullet_positions;
    utils::Locked<ISO3> odom_in_camera_cv;
    enum Type { AUTO_AIM, AUTO_BUFF } type = AUTO_AIM;
    utils::Locked<auto_aim::Armors> armors;
    utils::Locked<auto_aim::ArmorTarget> armor_target;
    utils::Locked<auto_aim::AutoAimFsm> auto_aim_fsm_state;

    utils::Locked<auto_buff::RuneDetection> rune;
    utils::Locked<auto_buff::RuneTarget> rune_target;

    utils::Locked<talos::ipc::GroundTruthBatch> ground_truth;
    CameraInfo camera_info() const noexcept {
        return camera_info_;
    }
};
class Web {
public:
    Web();
    AWAKENING_IMPL_DEFINITION(Web);
    void draw(cv::Mat& img, const VisionDebugCtx& ctx);
    void write_debug_data(const VisionDebugCtx& ctx);
    void write_shm(const cv::Mat& img);
    struct LogBuffer {
        std::mutex mtx;
        nlohmann::json j;
        bool dirty = false;

        std::ofstream file { "/dev/shm/awakening_log.json" };
        void flush() {
            static auto last_flush = std::chrono::steady_clock::now();

            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_flush).count() < 0.01)
                return;

            std::lock_guard<std::mutex> lock(mtx);

            if (!dirty)
                return;

            try {
                const std::string path = "/dev/shm/awakening_log.json";
                const std::string tmp_path = path + ".tmp";

                {
                    std::ofstream tmp_file(tmp_path, std::ios::out | std::ios::trunc);
                    if (!tmp_file.is_open())
                        return;

                    tmp_file << j.dump(2);
                    tmp_file.flush();
                }

                std::rename(tmp_path.c_str(), path.c_str());

                dirty = false;
                last_flush = now;

            } catch (...) {
            }
        }
    };
    template<typename T>
    static inline auto val(const T& v) {
        return +v;
    }
    static inline LogBuffer& get_log_buffer() {
        static LogBuffer buf;
        return buf;
    }
    template<typename Func>
    static inline void write_log(const char* key, Func&& f) {
        auto& buf = get_log_buffer();
        // static std::mutex mtx;
        {
            std::lock_guard<std::mutex> lock(buf.mtx);
            auto& j = buf.j[key];
            f(j);
            buf.dirty = true;
#ifdef USE_RERUN
            rerun_visual::Recorder::instance().log_json(key, j);
#endif
        }

        buf.flush();
    }
};

} // namespace awakening
