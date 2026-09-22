#pragma once
#include "angles.h"
#include "motion_model.hpp"
#include "tasks/auto_aim/type.hpp"
#include "utils/common/type_common.hpp"
#include <chrono>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace awakening::auto_aim {

struct ArmorTrackerCfg {
    int esekf_iter_num;
    double lost_time_thres;
    double lost_time_thres_outpost;
    int tracking_thres;
    double armor_match_gate;
    double armor_match_gate_not_all_init;
    double armor_match_w_center_err;
    double armor_match_w_angle_err;
    double armor_match_w_side_length_err;
    double qyaw_common;
    double qyaw_output;
    Vec3 qxyz_common;
    Vec3 qxyz_output;
    double q_r;
    double q_h;
    double q_wpr;
    double q_outpost_dz;
    double r_sigma_px_by_length_ratio;
    double r_sigma_length_by_length_ratio;
    double r_sigma_angle;
    double r_sigma_armor_lights_depth_diff;
    bool enable_lights_measure = false;
    double light_match_length_ratio_gate;
    double light_match_angle_gate;
    double light_match_pos_gate_by_length_ratio;
    void load(const YAML::Node& config) {
        esekf_iter_num = config["esekf_iter_num"].as<int>();
        lost_time_thres = config["lost_time_thres"].as<double>();
        lost_time_thres_outpost = config["lost_time_thres_outpost"].as<double>();
        tracking_thres = config["tracking_thres"].as<int>();
        armor_match_gate = config["armor_match_gate"].as<double>();
        armor_match_gate_not_all_init = config["armor_match_gate_not_all_init"].as<double>();
        armor_match_w_center_err = config["armor_match_w_center_err"].as<double>();
        armor_match_w_angle_err = config["armor_match_w_angle_err"].as<double>();
        armor_match_w_side_length_err = config["armor_match_w_side_length_err"].as<double>();
        qyaw_common = config["qyaw_common"].as<double>();
        qyaw_output = config["qyaw_output"].as<double>();
        auto qxyz_common_vec = config["qxyz_common"].as<std::vector<double>>();
        qxyz_common << qxyz_common_vec[0], qxyz_common_vec[1], qxyz_common_vec[2];
        auto qxyz_output_vec = config["qxyz_output"].as<std::vector<double>>();
        qxyz_output << qxyz_output_vec[0], qxyz_output_vec[1], qxyz_output_vec[2];
        q_r = config["q_r"].as<double>();
        q_h = config["q_h"].as<double>();
        q_wpr = config["q_wpr"].as<double>();
        q_outpost_dz = config["q_outpost_dz"].as<double>();
        r_sigma_angle = config["r_sigma_angle"].as<double>();
        r_sigma_px_by_length_ratio = config["r_sigma_px_by_length_ratio"].as<double>();
        r_sigma_length_by_length_ratio = config["r_sigma_length_by_length_ratio"].as<double>();
        r_sigma_armor_lights_depth_diff = config["r_sigma_armor_lights_depth_diff"].as<double>();
        enable_lights_measure = config["enable_lights_measure"].as<bool>();
        light_match_length_ratio_gate = config["light_match_length_ratio_gate"].as<double>();
        light_match_angle_gate = config["light_match_angle_gate"].as<double>();
        light_match_pos_gate_by_length_ratio =
            config["light_match_pos_gate_by_length_ratio"].as<double>();
    }
};
inline constexpr bool MEASURE_NORMALIZED = false;
inline int GLOBAL_ID = 0; // 全局状态标记，下游控制对同一 id 不重复构建轨迹。
class ArmorTarget {
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
    ArmorTarget() = default;
    static bool
    armor_pnp(Armor& a, const CameraInfo& camera_info, const ISO3& camera_cv_in_odom) noexcept;
    void reset(
        Armor& a,
        const ArmorTrackerCfg& c,
        const TimePoint& timestamp,
        int frame_id,
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
    [[nodiscard]] cv::Rect expanded(
        const TimePoint& timestamp,
        const ISO3& camera_cv_in_odom,
        const CameraInfo& camera_info,
        const cv::Size& image_size
    ) const noexcept;
    [[nodiscard]] std::vector<cv::Point2f> expanded_pts(
        const TimePoint& timestamp,
        const ISO3& camera_cv_in_odom,
        const CameraInfo& camera_info
    ) const noexcept;
    [[nodiscard]] Eigen::
        Matrix<double, armor_point_motion_model::X_N, armor_point_motion_model::X_N>
        process_noise(double dt) const noexcept;

    void predict_ekf(const TimePoint& timestamp);
    int update(
        std::vector<std::pair<int, Armor>>& a,
        std::vector<std::tuple<int, bool, Light>>& l,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    );
    std::pair<cv::Point2f, cv::Point2f> predict_light(
        int armor_id,
        bool is_left,
        const armor_point_motion_model::State& state,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) const noexcept;
    std::vector<std::pair<int, Armor>> match_armor(
        std::vector<Armor>& armors,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) const noexcept;
    std::vector<std::tuple<int, bool, Light>> match_light(
        std::vector<Light>& lights,
        const std::vector<std::pair<int, Armor>>& matched_armors,
        const std::vector<std::tuple<int, bool, std::pair<cv::Point2f, cv::Point2f>>>& visible_light
    ) const noexcept;
    std::vector<std::tuple<int, bool, Light>> match_light(
        std::vector<Light>& lights,
        const std::vector<std::pair<int, Armor>>& matched_armors,
        const TimePoint& timestamp,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) const noexcept;
    armor_point_motion_model::UVCtx uvmeasure_ctx;
    std::optional<armor_point_motion_model::RobotStateESEKF> esekf;
    ArmorTrackerCfg cfg;
    armor_point_motion_model::Voter voter;
    const armor_point_motion_model::State& get_target_state() const {
        return target_state;
    }
    template<typename F>
    void set_target_state(F&& f) {
        this_id = GLOBAL_ID++;
        f(target_state);
    }

    bool is_inited = false;
    bool jumped = false;
    int last_match_id = -1;
    std::optional<std::pair<bool, std::vector<bool>>> outpost_has_all_and_has_set_ids;
    TrackState track_state;
    TimePoint last_update;
    ArmorClass target_number = ArmorClass::UNKNOWN;
    int this_id = -1;
    int update_count = 0;
    [[nodiscard]] inline ArmorTarget fast_copy_without_ekf() const noexcept {
        ArmorTarget target;
        target.target_number = this->target_number;
        target.target_state = this->target_state;
        target.last_update = this->last_update;
        target.cfg = this->cfg;
        target.track_state = this->track_state;
        target.is_inited = this->is_inited;
        target.jumped = this->jumped;
        target.last_match_id = this->last_match_id;
        target.outpost_has_all_and_has_set_ids = this->outpost_has_all_and_has_set_ids;
        target.this_id = this->this_id;
        target.update_count = this->update_count;
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
    [[nodiscard]] inline bool need_detect_lights() const noexcept {
        return check() && cfg.enable_lights_measure;
    }
    [[nodiscard]] inline int armor_num() const noexcept {
        return armor_num_by_armor_class(target_number);
    }
    void write_log();

private:
    armor_point_motion_model::State target_state;
};
} // namespace awakening::auto_aim
