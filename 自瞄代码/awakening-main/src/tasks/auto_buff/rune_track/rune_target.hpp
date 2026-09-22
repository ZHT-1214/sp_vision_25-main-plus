#pragma once

#include "motion_model.hpp"
#include "tasks/auto_buff/type.hpp"
#include "utils/common/type_common.hpp"
#include <array>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
namespace awakening::auto_buff {
struct RuneTrackerCfg {
    int esekf_iter_num;
    double lost_time_thres;
    int tracking_thres;
    Vec3 q_xyz;
    double q_yaw;
    double q_roll;
    double q_a;
    double q_w;
    double q_tau;
    double r_sigma_pix_by_hand_length_ratio_cv;
    double r_sigma_pix_by_hand_length_ratio_net;
    double match_gate;
    int voter_state_need_count;
    int voter_mode_need_count;
    double big_args_continue_time;
    void load(const YAML::Node& config) {
        esekf_iter_num = config["esekf_iter_num"].as<int>();
        lost_time_thres = config["lost_time_thres"].as<double>();
        tracking_thres = config["tracking_thres"].as<int>();
        auto q_xyz_vec = config["q_xyz"].as<std::vector<double>>();
        q_xyz = Vec3(q_xyz_vec[0], q_xyz_vec[1], q_xyz_vec[2]);
        q_yaw = config["q_yaw"].as<double>();
        q_roll = config["q_roll"].as<double>();
        q_a = config["q_a"].as<double>();
        q_w = config["q_w"].as<double>();
        q_tau = config["q_tau"].as<double>();
        r_sigma_pix_by_hand_length_ratio_cv =
            config["r_sigma_pix_by_hand_length_ratio_cv"].as<double>();
        r_sigma_pix_by_hand_length_ratio_net =
            config["r_sigma_pix_by_hand_length_ratio_net"].as<double>();
        match_gate = config["match_gate"].as<double>();
        voter_state_need_count = config["voter_state_need_count"].as<int>();
        voter_mode_need_count = config["voter_mode_need_count"].as<int>();
        big_args_continue_time = config["big_args_continue_time"].as<double>();
    }
};
inline int GLOBAL_ID = 0; // 全局状态标记，下游控制对同一 id 不重复构建轨迹。
struct FanWC {
    std::array<bool, FAN_NUM> is_visible {};
    std::array<TimePoint, FAN_NUM> fan_times;
    void reset() {
        is_visible.fill(false);
    }
    void update(int i, const TimePoint& timestamp) {
        is_visible[i] = true;
        fan_times[i] = timestamp;
    }
    int get_min_visable_fan_id() const {
        for (int i = 0; i < FAN_NUM; ++i) {
            if (is_visible[i]) {
                return i;
            }
        }
        return 0;
    }
    std::string to_str() const {
        std::string str;
        for (int i = 0; i < FAN_NUM; ++i) {
            if (is_visible[i]) {
                str += std::to_string(i) + " ";
            }
        }
        return str;
    }
};
class RuneTarget {
public:
    struct TrackState {
        enum State {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
        };
        State tracker_state = LOST;
        int detect_count = 0;
        int lost_count = 0;
        static inline std::string string_by_state(State state) {
            constexpr const char* details[] = { "LOST", "DETECTING", "TRACKING", "TEMP_LOST" };
            return std::string(details[state]);
        }
        bool is_tracking() const noexcept {
            return tracker_state == TRACKING || tracker_state == TEMP_LOST;
        }
        void reset() {
            tracker_state = LOST;
            detect_count = 0;
            lost_count = 0;
        }
    };
    RuneTarget() = default;
    bool reset(
        RuneDetection& d,
        const RuneTrackerCfg& c,
        const TimePoint& timestamp,
        int frame_id,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    );
    static void fan_pnp(
        RuneFanBladeWithR& a,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom,
        bool in_r
    ) noexcept;
    static void fan_target_pnp(
        RuneFanTarget& a,
        const cv::Point2f& r,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom,
        bool in_r
    ) noexcept;
    [[nodiscard]] Eigen::Matrix<double, motion_model::X_N, motion_model::X_N>
    process_noise(double dt) const noexcept;
    [[nodiscard]] Eigen::Matrix<double, motion_model::YPDZ_N, motion_model::YPDZ_N>
    ypdmeasurement_covariance(const Eigen::Matrix<double, motion_model::YPDZ_N, 1>& z
    ) const noexcept;
    [[nodiscard]] Eigen::Matrix<double, motion_model::YPDZ_N, 1> get_ypdmeasurement(const ISO3& pose
    ) const noexcept;
    void predict_ekf(const TimePoint& timestamp);
    std::vector<std::pair<int, RuneFanBladeWithR>> match_fan(
        std::vector<RuneFanBladeWithR>& fans,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) const noexcept;
    std::optional<std::pair<bool, cv::Point2f>> match_r(
        const std::vector<std::pair<int, RuneFanBladeWithR>>& matched_fans,
        std::vector<RuneR>& rs,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    );
    std::vector<std::pair<int, RuneFanTarget>> match_fan_target(
        std::vector<RuneFanTarget>& fans,
        const std::optional<std::pair<bool, cv::Point2f>>& r,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) const noexcept;
    std::tuple<int, int> update(
        const std::vector<std::pair<int, RuneFanBladeWithR>>& f,
        const std::vector<std::pair<int, RuneFanTarget>>& a,
        const std::optional<std::pair<bool, cv::Point2f>>& r,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    );
    [[nodiscard]] cv::Rect get_net_focus_roi(
        const TimePoint& timestamp,
        const ISO3& camera_cv_in_odom,
        const CameraInfo& camera_info,
        const cv::Size& image_size,
        double target_wh_ratio = 1.0
    ) const noexcept;

    const motion_model::State& get_target_state() const {
        return target_state;
    }
    template<typename F>
    void set_target_state(F&& f) {
        this_id = GLOBAL_ID++;
        f(target_state);
    }
    [[nodiscard]] inline RuneTarget fast_copy_without_ekf() const noexcept {
        RuneTarget target;
        target.target_state = this->target_state;
        target.last_update = this->last_update;
        target.cfg = this->cfg;
        target.track_state = this->track_state;
        target.is_inited = this->is_inited;
        target.this_id = this->this_id;
        target.fan_wc = this->fan_wc;
        target.voter = this->voter;
        return target;
    }
    [[nodiscard]] inline bool check() const noexcept {
        return track_state.is_tracking()
            && std::chrono::duration<double>(Clock::now() - last_update).count()
            < cfg.lost_time_thres;
    }
    [[nodiscard]] inline bool need_focus() const noexcept {
        return is_inited
            && std::chrono::duration<double>(Clock::now() - last_update).count()
            < cfg.lost_time_thres;
    }
    FanWC get_fan_wc() const noexcept {
        return fan_wc;
    }
    void write_log();
    std::optional<motion_model::ESEKF> esekf;
    motion_model::RMeasure::Ctx r_ctx;
    motion_model::FanBladeMeasure::Ctx fan_ctx;
    motion_model::FanTargetMeasure::Ctx fan_target_ctx;
    motion_model::Voter voter;
    RuneTrackerCfg cfg;
    TimePoint last_update;
    TrackState track_state;
    int this_id = -1;
    bool is_inited = false;
    FanWC fan_wc;

private:
    motion_model::State target_state;
};
} // namespace awakening::auto_buff
