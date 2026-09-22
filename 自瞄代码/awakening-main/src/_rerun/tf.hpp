#pragma once

#include "_rerun/recorder.hpp"
#include "utils/runtime_tf.hpp"

namespace awakening::rerun_visual {

template<typename FrameEnum, size_t N, bool Static, typename F>
inline void log_robot_tf(const utils::tf::RobotTF<FrameEnum, N, Static>& tf, F&& frame_name) {
    auto& rec = Recorder::instance();
    if (!rec.enabled()) {
        return;
    }
    for (const auto& edge: tf.get_edges()) {
        const auto parent = static_cast<FrameEnum>(edge.parent);
        const auto child = static_cast<FrameEnum>(edge.child);
        rec.log_transform(
            frame_name(parent),
            frame_name(child),
            tf.pose_a_in_b(child, parent, Clock::now())
        );
    }
}

} // namespace awakening::rerun_visual
