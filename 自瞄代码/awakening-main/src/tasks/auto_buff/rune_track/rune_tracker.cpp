#include "rune_tracker.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/dta_utils.hpp"
#include "utils/logger.hpp"
#include <memory>
namespace awakening::auto_buff {
struct RuneTracker::Impl {
    Impl(const YAML::Node& config) {
        cfg_.load(config);
    }
    RuneTarget track(
        RuneDetection& detection,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom,
        int frame_id
    ) {
        const bool found = target_.track_state.tracker_state == RuneTarget::TrackState::LOST
            ? init_target(detection, frame_id, camera_info, camera_cv_in_odom)
            : update_target(detection, camera_info, camera_cv_in_odom);
        if (target_.get_target_state().pos().norm() > 15.0 && target_.check()) {
            target_.track_state.tracker_state = RuneTarget::TrackState::LOST;
            AWAKENING_WARN("TOO FAR");
        }
        update_fsm(found, detection.timestamp);
        // detection.rune_rs.erase(
        //     std::remove_if(
        //         detection.rune_rs.begin(),
        //         detection.rune_rs.end(),
        //         [](const RuneR& r) { return r.laji; }
        //     ),
        //     detection.rune_rs.end()
        // );
        return target_.fast_copy_without_ekf();
    }
    bool init_target(
        RuneDetection& r,
        int frame_id,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) noexcept {
        if (!target_.reset(r, cfg_, r.timestamp, frame_id, camera_info, camera_cv_in_odom)) {
            return false;
        }
        AWAKENING_INFO("init rune target");
        target_.track_state.tracker_state = RuneTarget::TrackState::DETECTING;
        return true;
    }
    bool update_target(
        RuneDetection& r,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) noexcept {
        if (r.fan_blades.empty() && r.rune_rs.empty() && r.fan_targets.empty()) {
            return false;
        }
        target_.predict_ekf(r.timestamp);
        auto matched_fans =
            target_.match_fan(r.fan_blades, r.timestamp, camera_info, camera_cv_in_odom);
        auto match_r =
            target_.match_r(matched_fans, r.rune_rs, r.timestamp, camera_info, camera_cv_in_odom);
        auto matched_fan_targets = target_.match_fan_target(
            r.fan_targets,
            match_r,
            r.timestamp,
            camera_info,
            camera_cv_in_odom
        );
        const auto updated = target_.update(
            matched_fans,
            matched_fan_targets,
            match_r,
            r.timestamp,
            camera_info,
            camera_cv_in_odom
        );
        return std::get<0>(updated) > 0;
    }
    void update_fsm(bool found, const TimePoint& now) noexcept {
        auto& target = target_;
        auto& s = target.track_state;
        if (found)
            ++found_count_;
        dta_utils::update_fsm(
            found,
            s,
            cfg_.tracking_thres,
            dta_utils::elapsed_sec(target.last_update, now),
            cfg_.lost_time_thres
        );
    }
    RuneTrackerCfg cfg_;
    RuneTarget target_;
    int found_count_ = 0;
};
RuneTracker::RuneTracker(const YAML::Node& config): _impl(std::make_unique<Impl>(config)) {}
RuneTracker::~RuneTracker() noexcept = default;
RuneTarget RuneTracker::track(
    RuneDetection& detection,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom,
    int frame_id
) {
    return _impl->track(detection, camera_info, camera_cv_in_odom, frame_id);
}
int RuneTracker::get_count() {
    return _impl->found_count_;
}
void RuneTracker::reset_count() {
    _impl->found_count_ = 0;
}
} // namespace awakening::auto_buff
