#include "armor_tracker.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/dta_utils.hpp"
#include "utils/logger.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>
namespace awakening::auto_aim {
struct ArmorTracker::Impl {
    Impl(const YAML::Node& config) {
        cfg_.load(config);
    }
    ArmorTarget track(
        Armors& armors,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom,
        int frame_id
    ) {
        auto& cur = target_buf_[cur_target_idx_];
        auto& pre = target_buf_[pre_target_idx_];
        auto process = [&](int idx) {
            auto& t = target_buf_[idx];
            bool found = (t.track_state.tracker_state == ArmorTarget::TrackState::LOST)
                ? init_target(t, armors, frame_id, camera_info, camera_cv_in_odom)
                : update_target(t, armors, camera_info, camera_cv_in_odom);
            update_fsm(found, idx, armors.timestamp);
            return found;
        };
        // 双缓冲，方便异常丢失恢复和操作手换目标。
        process(cur_target_idx_);

        if (cur.track_state.tracker_state == ArmorTarget::TrackState::TEMP_LOST) {
            process(pre_target_idx_);

            if (pre.track_state.tracker_state == ArmorTarget::TrackState::TRACKING) {
                std::swap(cur, pre);
                pre.track_state.tracker_state = ArmorTarget::TrackState::LOST;
            }
        } else if (cur.track_state.tracker_state == ArmorTarget::TrackState::TRACKING) {
            pre.track_state.tracker_state = ArmorTarget::TrackState::LOST;
        }
        armors.lights.erase(
            std::remove_if(
                armors.lights.begin(),
                armors.lights.end(),
                [](const Light& l) { return l.laji; }
            ),
            armors.lights.end()
        );

        return target_buf_[cur_target_idx_].fast_copy_without_ekf(); //下游不让用ekf
    }
    bool init_target(
        ArmorTarget& target,
        Armors& armors,
        int frame_id,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) noexcept {
        if (armors.armors.empty()) {
            return false;
        }
        const auto valid_target = [&](Armor& armor) {
            const bool valid_color =
                armor.color != ArmorColor::NONE && armor.color != ArmorColor::PURPLE;
            const bool keep_outpost =
                target_buf_[cur_target_idx_].target_number == ArmorClass::OUTPOST
                && armor.number != ArmorClass::OUTPOST && target_buf_[cur_target_idx_].check();
            const bool pnp_ok = ArmorTarget::armor_pnp(armor, camera_info, camera_cv_in_odom);
            return valid_color && !keep_outpost && pnp_ok;
        };
        Armor* selected_armor = nullptr;
        auto min_dis = std::numeric_limits<double>::max();
        for (auto& armor: armors.armors) {
            if (valid_target(armor)) {
                auto bbox = armor.key_points.bounding_box();
                auto dis = utils::calculate_distance_to_img_center(
                    (bbox.tl() + bbox.br()) / 2.0,
                    camera_info.camera_matrix
                );
                if (dis < min_dis) {
                    min_dis = dis;
                    selected_armor = &armor;
                }
            }
        }
        if (!selected_armor) {
            return false;
        }

        Armor init_target = *selected_armor;
        AWAKENING_INFO("init target: {}", string_by_armor_class(init_target.number));
        target.reset(init_target, cfg_, armors.timestamp, frame_id, camera_info, camera_cv_in_odom);
        target.track_state.tracker_state = ArmorTarget::TrackState::DETECTING;
        return true;
    }
    bool update_target(
        ArmorTarget& target,
        Armors& armors,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) noexcept {
        std::vector<Armor> candidates;
        candidates.reserve(armors.armors.size());
        for (const auto& a: armors.armors) {
            if (a.number == target.target_number && a.color != ArmorColor::NONE
                && a.color != ArmorColor::PURPLE)
            {
                candidates.emplace_back(a);
            }
            // if (a.number == target.target_number)
            // {
            //     candidates.emplace_back(a);
            // }
        }
        target.predict_ekf(armors.timestamp);
        auto matched_armors =
            target.match_armor(candidates, armors.timestamp, camera_info, camera_cv_in_odom);
        auto matched_lights = target.match_light(
            armors.lights,
            matched_armors,
            armors.timestamp,
            camera_info,
            camera_cv_in_odom
        );
        const int updated = target.update(
            matched_armors,
            matched_lights,
            armors.timestamp,
            camera_info,
            camera_cv_in_odom
        );
        return updated > 0;
    }
    void update_fsm(bool found, size_t i, const TimePoint& now) noexcept {
        auto& target = target_buf_[i];
        auto& s = target.track_state;
        if (found)
            ++found_count_;
        dta_utils::update_fsm(
            found,
            s,
            cfg_.tracking_thres,
            dta_utils::elapsed_sec(target.last_update, now),
            lost_time_thres(target)
        );
    }
    double lost_time_thres(const ArmorTarget& target) const noexcept {
        return (target.target_number == ArmorClass::OUTPOST) ? cfg_.lost_time_thres_outpost
                                                             : cfg_.lost_time_thres;
    }
    void set_sentry(bool is_sentry) {
        iam_sentry = is_sentry;
    }

    int found_count_ = 0;

    size_t cur_target_idx_ = 0;
    size_t pre_target_idx_ = 1;
    std::array<ArmorTarget, 2> target_buf_;
    ArmorTrackerCfg cfg_;
    bool iam_sentry = false;
};
ArmorTracker::ArmorTracker(const YAML::Node& config): _impl(std::make_unique<Impl>(config)) {}
ArmorTracker::~ArmorTracker() noexcept = default;

ArmorTarget ArmorTracker::track(
    Armors& armors,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom,
    int frame_id
) {
    return _impl->track(armors, camera_info, camera_cv_in_odom, frame_id);
}
int ArmorTracker::get_count() {
    return _impl->found_count_;
}
void ArmorTracker::reset_count() {
    _impl->found_count_ = 0;
}
void ArmorTracker::set_sentry(bool is_sentry) {
    _impl->set_sentry(is_sentry);
}
} // namespace awakening::auto_aim
