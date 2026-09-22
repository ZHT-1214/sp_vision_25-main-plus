#pragma once

#include "angles.h"
#include "dual_small_mpc_solver.hpp"
#include "tasks/base/common.hpp"
#include "tasks/base/traj.hpp"
#include "tinympc/tiny_api.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <tbb/parallel_invoke.h>
#include <utility>
#include <vector>
namespace awakening::dta_utils {
template<typename TrackState>
inline void update_fsm(
    bool found,
    TrackState& state,
    int tracking_thres,
    double lost_time,
    double lost_time_thres
) noexcept {
    switch (state.tracker_state) {
        case TrackState::DETECTING:
            if (!found) {
                state.detect_count = 0;
                state.tracker_state = TrackState::LOST;
                return;
            }
            if (++state.detect_count > tracking_thres) {
                state.detect_count = 0;
                state.tracker_state = TrackState::TRACKING;
            }
            return;

        case TrackState::TRACKING:
            if (!found) {
                state.tracker_state = TrackState::TEMP_LOST;
            }
            return;

        case TrackState::TEMP_LOST:
            if (found) {
                state.tracker_state = TrackState::TRACKING;
                return;
            }
            if (lost_time > lost_time_thres) {
                state.tracker_state = TrackState::LOST;
            }
            return;

        default:
            return;
    }
}

inline double elapsed_sec(const TimePoint& from, const TimePoint& to) noexcept {
    return std::max(0.0, std::chrono::duration<double>(to - from).count());
}

template<typename CostMatrix>
inline std::vector<std::pair<int, int>>
greedy_match(const CostMatrix& cost, int n_obs, int n_ids, double max_cost) {
    std::vector<std::pair<int, int>> result;
    std::vector<bool> used_obs(n_obs, false);
    std::vector<bool> used_id(n_ids, false);

    while (true) {
        double best = max_cost;
        int best_obs = -1;
        int best_id = -1;

        for (int obs = 0; obs < n_obs; ++obs) {
            if (used_obs[obs]) {
                continue;
            }
            for (int id = 0; id < n_ids; ++id) {
                if (used_id[id]) {
                    continue;
                }
                if (cost[obs][id] < best) {
                    best = cost[obs][id];
                    best_obs = obs;
                    best_id = id;
                }
            }
        }

        if (best_obs < 0 || best_id < 0) {
            break;
        }

        used_obs[best_obs] = true;
        used_id[best_id] = true;
        result.emplace_back(best_obs, best_id);
    }

    return result;
}

struct ControlPoint {
    double yaw;
    double pitch;
    int aim_id;
    AimPoint aim_point;
    bool valid;
};
struct GimbalState {
    struct State {
        double p;
        double v;
        double a;
        bool on_traj;
        static State lerp(const State& s0, const State& s1, double a) noexcept {
            State r { .p = utils::lerp_angle(s0.p, s1.p, a),
                      .v = std::lerp(s0.v, s1.v, a),
                      .a = std::lerp(s0.a, s1.a, a) };

            return r;
        }
    };
    State yaw_state;
    State pitch_state;
    int aim_id = 0;
    GimbalState() = default;
    GimbalState(const GimbalState::State& y, const GimbalState::State& p):
        yaw_state(y),
        pitch_state(p) {}
    static GimbalState lerp(const GimbalState& s0, const GimbalState& s1, double a) noexcept {
        GimbalState r;
        r.aim_id = (a < 0.5) ? s0.aim_id : s1.aim_id;
        r.yaw_state = State::lerp(s0.yaw_state, s1.yaw_state, a);
        r.pitch_state = State::lerp(s0.pitch_state, s1.pitch_state, a);
        return r;
    }
};
template<class Scale>
struct QuinticSegment {
    Scale T = 0.0;
    Eigen::Matrix<Scale, 6, 1> c;
    GimbalState::State head;
    GimbalState::State tail;
    bool on_traj;

    static inline Eigen::Matrix<Scale, 6, 1> solve1d_closed_form(
        Scale p0,
        Scale v0,
        Scale a0,
        Scale p1,
        Scale v1,
        Scale a1,
        Scale T
    ) noexcept {
        Eigen::Matrix<Scale, 6, 1> c;
        c.setZero();

        if (T <= static_cast<Scale>(1e-9)) {
            c[0] = p0;
            return c;
        }

        const Scale invT = static_cast<Scale>(1.0) / T;
        const Scale invT2 = invT * invT;
        const Scale invT3 = invT2 * invT;
        const Scale invT4 = invT3 * invT;
        const Scale invT5 = invT4 * invT;

        // c0, c1, c2
        c[0] = p0;
        c[1] = v0;
        c[2] = static_cast<Scale>(0.5) * a0;

        // boundary mismatch
        const Scale dp = p1 - (p0 + v0 * T + static_cast<Scale>(0.5) * a0 * T * T);
        const Scale dv = v1 - (v0 + a0 * T);
        const Scale da = a1 - a0;

        // quintic coefficients
        c[3] = (static_cast<Scale>(10.0) * dp - static_cast<Scale>(4.0) * dv * T
                + static_cast<Scale>(0.5) * da * T * T)
            * invT3;

        c[4] = (static_cast<Scale>(-15.0) * dp + static_cast<Scale>(7.0) * dv * T
                - static_cast<Scale>(1.0) * da * T * T)
            * invT4;

        c[5] = (static_cast<Scale>(6.0) * dp - static_cast<Scale>(3.0) * dv * T
                + static_cast<Scale>(0.5) * da * T * T)
            * invT5;

        return c;
    }

    [[nodiscard]] static inline QuinticSegment build(
        const GimbalState::State& s0,
        const GimbalState::State& s1,
        Scale T,
        bool on_traj
    ) noexcept {
        QuinticSegment seg;
        seg.head = s0;
        seg.tail = s1;
        seg.T = T;
        seg.c = solve1d_closed_form(s0.p, s0.v, s0.a, s1.p, s1.v, s1.a, T);
        seg.on_traj = on_traj;
        return seg;
    }

    static inline Scale max_abs_acc(const Eigen::Matrix<Scale, 6, 1>& c, Scale T) noexcept {
        if (T <= static_cast<Scale>(0))
            return static_cast<Scale>(0);

        auto acc = [&](Scale t) {
            Scale t2 = t * t;
            return 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t2 + 20 * c[5] * t2 * t;
        };

        Scale max_acc = std::max(std::abs(acc(0.0)), std::abs(acc(T)));

        const Scale eps = static_cast<Scale>(1e-9);

        const Scale A = 60.0 * c[5];
        const Scale B = 24.0 * c[4];
        const Scale C = 6.0 * c[3];

        auto update = [&](Scale t) {
            if (t > 0.0 && t < T) {
                max_acc = std::max(max_acc, std::abs(acc(t)));
            }
        };

        if (std::abs(A) < eps) {
            if (std::abs(B) > eps) {
                update(-C / B);
            }
            // else: jerk ~ constant → only endpoints matter
        } else {
            Scale D = B * B - 4 * A * C;

            if (D > eps) {
                Scale sqrtD = std::sqrt(D);
                Scale inv2A = static_cast<Scale>(0.5) / A;

                update((-B + sqrtD) * inv2A);
                update((-B - sqrtD) * inv2A);
            }
        }

        return std::isfinite(max_acc) ? max_acc : static_cast<Scale>(0);
    }

    [[nodiscard]] Scale inline duration() const noexcept {
        return T;
    }

    [[nodiscard]] Scale inline max_acc() const noexcept {
        return QuinticSegment::max_abs_acc(c, T);
    }
    [[nodiscard]] GimbalState::State inline eval(Scale t) const noexcept {
        GimbalState::State s;
        if (T <= 0.0)
            return s;
        t = std::clamp<Scale>(t, 0.0, T);
        Scale t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        s.p = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
        s.v = c[1] + 2 * c[2] * t + 3 * c[3] * t2 + 4 * c[4] * t3 + 5 * c[5] * t4;
        s.a = 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t2 + 20 * c[5] * t3;
        s.on_traj = on_traj;
        return s;
    }
};

class LimitTrajectory: public Trajectory<GimbalState, double> {
public:
    using Seg = QuinticSegment<double>;
    struct Traj {
        std::vector<Seg> segs;
        std::vector<int> seg_start_idx;
        std::vector<int> seg_end_idx;
        std::vector<double> seg_prefix_time;
        std::optional<std::pair<int, int>> limit_interval;
        void reserve(size_t size) {
            segs.reserve(size);
            seg_start_idx.reserve(size);
            seg_end_idx.reserve(size);
            seg_prefix_time.reserve(size + 1);
        }
        void clear() {
            segs.clear();
            seg_start_idx.clear();
            seg_end_idx.clear();
            seg_prefix_time.clear();
            limit_interval.reset();
        }
        void push_seg(Seg seg, int start_idx, int end_idx) {
            segs.push_back(std::move(seg));
            seg_start_idx.push_back(start_idx);
            seg_end_idx.push_back(end_idx);
        }
        void rebuild_prefix(double first_time) {
            seg_prefix_time.resize(segs.size() + 1);
            seg_prefix_time[0] = first_time;
            for (size_t i = 0; i < segs.size(); ++i) {
                seg_prefix_time[i + 1] = seg_prefix_time[i] + segs[i].duration();
            }
        }
    };

    Traj yaw_traj;
    Traj pitch_traj;

    void unwrap_states(std::vector<GimbalState>& s) const noexcept {
        if (s.size() < 2)
            return;
        for (size_t i = 1; i < s.size(); ++i) {
            s[i].yaw_state.p = angles::unwrap_angle(s[i - 1].yaw_state.p, s[i].yaw_state.p);
            s[i].pitch_state.p = angles::unwrap_angle(s[i - 1].pitch_state.p, s[i].pitch_state.p);
        }
    }
    void clear() {
        Trajectory::clear();
        yaw_traj.clear();
        pitch_traj.clear();
    }

    struct SegmentDesc {
        int l = 0;
        int r = 0;
        bool on_traj = true;
    };

    [[nodiscard]] static inline double segment_avg_v(
        const std::vector<GimbalState::State>& s,
        const std::vector<double>& prefix,
        const SegmentDesc& d
    ) noexcept {
        const double T = prefix[d.r] - prefix[d.l];
        return T > 1e-9 ? (s[d.r].p - s[d.l].p) / T : 0.0;
    }

    [[nodiscard]] static inline std::vector<GimbalState::State> estimate_knot_states(
        const std::vector<GimbalState::State>& s,
        const std::vector<double>& prefix,
        const std::vector<SegmentDesc>& descs
    ) noexcept {
        std::vector<GimbalState::State> knots = s;
        for (auto& state: knots) {
            state.v = 0.0;
            state.a = 0.0;
        }

        std::vector<double> left_v(s.size(), 0.0);
        std::vector<double> right_v(s.size(), 0.0);
        std::vector<double> left_T(s.size(), 0.0);
        std::vector<double> right_T(s.size(), 0.0);
        std::vector<bool> has_left(s.size(), false);
        std::vector<bool> has_right(s.size(), false);

        for (const auto& d: descs) {
            if (!d.on_traj)
                continue;

            const double T = prefix[d.r] - prefix[d.l];
            if (T <= 1e-9)
                continue;

            const double avg_v = segment_avg_v(s, prefix, d);
            right_v[d.l] = avg_v;
            right_T[d.l] = T;
            has_right[d.l] = true;

            left_v[d.r] = avg_v;
            left_T[d.r] = T;
            has_left[d.r] = true;
        }

        for (size_t i = 0; i < knots.size(); ++i) {
            if (!has_left[i] || !has_right[i])
                continue;

            const double T = left_T[i] + right_T[i];
            if (T <= 1e-9)
                continue;

            knots[i].v = (right_T[i] * left_v[i] + left_T[i] * right_v[i]) / T;
            knots[i].a = 2.0 * (right_v[i] - left_v[i]) / T;
        }

        return knots;
    }

    [[nodiscard]] std::vector<SegmentDesc>
    make_segment_descs(int N, const std::optional<std::pair<int, int>>& interval) const {
        std::vector<SegmentDesc> descs;
        if (N <= 1)
            return descs;

        descs.reserve(N - 1);
        for (int i = 0; i < N - 1; ++i) {
            if (interval && i == interval->first) {
                descs.push_back({ interval->first, interval->second, false });
                i = interval->second - 1;
            } else {
                descs.push_back({ i, i + 1, true });
            }
        }
        return descs;
    }

    [[nodiscard]] std::optional<std::pair<int, int>> find_nearest_change_interval(
        const std::vector<GimbalState>& cp_vec,
        const std::vector<double>& prefix,
        double current_time
    ) const noexcept {
        std::optional<std::pair<int, int>> interval;
        double best_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i + 1 < cp_vec.size(); ++i) {
            if (cp_vec[i].aim_id == cp_vec[i + 1].aim_id)
                continue;

            const double seg_mid = 0.5 * (prefix[i] + prefix[i + 1]);
            const double dist = std::abs(seg_mid - current_time);
            if (dist < best_dist) {
                best_dist = dist;
                interval.emplace(static_cast<int>(i), static_cast<int>(i + 1));
            }
        }
        return interval;
    }

    [[nodiscard]] std::optional<std::pair<int, int>> expand_limit_interval(
        const std::vector<GimbalState>& cp_vec,
        const std::vector<GimbalState::State>& s,
        const std::vector<double>& prefix,
        std::optional<std::pair<int, int>> interval,
        double max_acc
    ) const noexcept {
        const int N = static_cast<int>(s.size());
        if (!interval)
            return interval;

        auto buildSeg = [&](int l, int r) -> Seg {
            double dur = prefix[r] - prefix[l];
            return Seg::build(s[l], s[r], dur, false);
        };
        const int base_l = interval->first;
        const int base_r = interval->second;

        int left_run_start = base_l;
        while (left_run_start > 0
               && cp_vec[left_run_start - 1].aim_id == cp_vec[left_run_start].aim_id) {
            --left_run_start;
        }

        int right_run_end = base_r;
        while (right_run_end + 1 < N
               && cp_vec[right_run_end].aim_id == cp_vec[right_run_end + 1].aim_id) {
            ++right_run_end;
        }

        const double left_mid_time = 0.5 * (prefix[left_run_start] + prefix[base_l]);
        const auto left_limit_it = std::lower_bound(
            prefix.begin() + left_run_start,
            prefix.begin() + base_l + 1,
            left_mid_time
        );
        const int left_limit = static_cast<int>(std::distance(prefix.begin(), left_limit_it));

        const double right_mid_time = 0.5 * (prefix[base_r] + prefix[right_run_end]);
        const auto right_limit_it = std::upper_bound(
            prefix.begin() + base_r,
            prefix.begin() + right_run_end + 1,
            right_mid_time
        );
        const int right_limit = static_cast<int>(std::distance(prefix.begin(), right_limit_it)) - 1;

        auto radius_interval = [&](int radius) -> std::pair<int, int> {
            return { std::max(left_limit, base_l - radius),
                     std::min(right_limit, base_r + radius) };
        };

        auto acc_at_radius = [&](int radius) -> double {
            const auto [l, r] = radius_interval(radius);
            return buildSeg(l, r).max_acc();
        };

        if (acc_at_radius(0) > max_acc) {
            const int max_radius = std::max(base_l - left_limit, right_limit - base_r);
            int best_radius = max_radius;

            for (int radius = 1; radius <= max_radius; ++radius) {
                if (acc_at_radius(radius) <= max_acc) {
                    best_radius = radius;
                    break;
                }
            }

            interval = radius_interval(best_radius);
        }
        return interval;
    }

    void build_continuous_centered_traj(
        Traj& traj,
        const std::vector<GimbalState::State>& s,
        const std::vector<double>& prefix,
        const std::vector<SegmentDesc>& descs
    ) const noexcept {
        traj.segs.clear();
        traj.seg_start_idx.clear();
        traj.seg_end_idx.clear();
        traj.seg_prefix_time.clear();
        if (descs.empty())
            return;

        const auto knots = estimate_knot_states(s, prefix, descs);
        auto duration = [&](const SegmentDesc& d) { return prefix[d.r] - prefix[d.l]; };

        traj.segs.reserve(descs.size());
        traj.seg_start_idx.reserve(descs.size());
        traj.seg_end_idx.reserve(descs.size());
        for (size_t i = 0; i < descs.size(); ++i) {
            const auto& d = descs[i];
            traj.push_seg(Seg::build(knots[d.l], knots[d.r], duration(d), d.on_traj), d.l, d.r);
        }
        traj.rebuild_prefix(prefix[descs.front().l]);
    }

    template<typename ProjectState>
    void limit_traj(
        Traj& traj,
        const std::vector<GimbalState>& cp_vec,
        const std::vector<double>& prefix,
        std::optional<std::pair<int, int>> change_interval,
        double max_acc,
        ProjectState&& project
    ) const noexcept {
        traj.clear();

        const int N = static_cast<int>(cp_vec.size());
        if (N <= 1)
            return;

        std::vector<GimbalState::State> s;
        s.resize(cp_vec.size());
        for (size_t i = 0; i < cp_vec.size(); ++i) {
            s[i] = project(cp_vec[i]);
            s[i].v = 0.0;
            s[i].a = 0.0;
        }

        const auto interval = expand_limit_interval(cp_vec, s, prefix, change_interval, max_acc);
        traj.limit_interval = interval;
        const auto descs = make_segment_descs(N, interval);
        build_continuous_centered_traj(traj, s, prefix, descs);
    }

    void build_limit(double max_yaw_acc, double max_pitch_acc, double current_time) noexcept {
        auto& cp_vec = get_cp_vec();
        const auto& prefix = get_prefix();
        unwrap_states(cp_vec);
        const int N = static_cast<int>(cp_vec.size());
        if (N < 2)
            return;
        const auto change_interval = find_nearest_change_interval(cp_vec, prefix, current_time);
        limit_traj(
            yaw_traj,
            cp_vec,
            prefix,
            change_interval,
            max_yaw_acc,
            [](const GimbalState& s) { return s.yaw_state; }
        );
        limit_traj(
            pitch_traj,
            cp_vec,
            prefix,
            change_interval,
            max_pitch_acc,
            [](const GimbalState& s) { return s.pitch_state; }
        );
    }

    void update_limit_after_append(
        double max_yaw_acc,
        double max_pitch_acc,
        double current_time,
        size_t old_size
    ) noexcept {
        auto& cp_vec = get_cp_vec();
        const auto& prefix = get_prefix();
        const size_t N = cp_vec.size();
        if (N < 2) {
            return;
        }
        if (old_size >= N) {
            return;
        }
        if (old_size < 2 || yaw_traj.segs.empty() || pitch_traj.segs.empty()) {
            build_limit(max_yaw_acc, max_pitch_acc, current_time);
            return;
        }

        unwrap_appended_states(cp_vec, old_size);

        const int first_tail_idx = static_cast<int>(old_size) - 2;
        const auto change_interval = find_nearest_change_interval(cp_vec, prefix, current_time);
        if (interval_touches_tail(change_interval, first_tail_idx)
            || interval_touches_tail(yaw_traj.limit_interval, first_tail_idx)
            || interval_touches_tail(pitch_traj.limit_interval, first_tail_idx))
        {
            build_limit(max_yaw_acc, max_pitch_acc, current_time);
            return;
        }

        append_unlimited_tail(yaw_traj, cp_vec, prefix, first_tail_idx, [](const GimbalState& s) {
            return s.yaw_state;
        });
        append_unlimited_tail(pitch_traj, cp_vec, prefix, first_tail_idx, [](const GimbalState& s) {
            return s.pitch_state;
        });
    }

private:
    void unwrap_appended_states(std::vector<GimbalState>& cp_vec, size_t old_size) const noexcept {
        if (cp_vec.size() < 2)
            return;
        size_t begin = old_size == 0 ? 1 : old_size;
        begin = std::max<size_t>(begin, 1);
        for (size_t i = begin; i < cp_vec.size(); ++i) {
            cp_vec[i].yaw_state.p =
                angles::unwrap_angle(cp_vec[i - 1].yaw_state.p, cp_vec[i].yaw_state.p);
            cp_vec[i].pitch_state.p =
                angles::unwrap_angle(cp_vec[i - 1].pitch_state.p, cp_vec[i].pitch_state.p);
        }
    }

    [[nodiscard]] static bool interval_touches_tail(
        const std::optional<std::pair<int, int>>& interval,
        int first_tail_idx
    ) noexcept {
        return interval && interval->second >= first_tail_idx;
    }

    template<typename ProjectState>
    void append_unlimited_tail(
        Traj& traj,
        const std::vector<GimbalState>& cp_vec,
        const std::vector<double>& prefix,
        int first_tail_idx,
        ProjectState&& project
    ) const noexcept {
        const int N = static_cast<int>(cp_vec.size());
        if (N <= 1)
            return;

        first_tail_idx = std::clamp(first_tail_idx, 0, N - 2);
        const int first_knot_idx = std::max(0, first_tail_idx - 1);

        std::vector<GimbalState::State> s(cp_vec.size());
        for (size_t i = 0; i < cp_vec.size(); ++i) {
            s[i] = project(cp_vec[i]);
            s[i].v = 0.0;
            s[i].a = 0.0;
        }

        std::vector<SegmentDesc> knot_descs;
        knot_descs.reserve(static_cast<size_t>(N - first_knot_idx - 1));
        for (int i = first_knot_idx; i < N - 1; ++i) {
            knot_descs.push_back({ i, i + 1, true });
        }
        const auto knots = estimate_knot_states(s, prefix, knot_descs);

        auto first_removed =
            std::lower_bound(traj.seg_start_idx.begin(), traj.seg_start_idx.end(), first_tail_idx);
        const size_t first_removed_idx =
            static_cast<size_t>(std::distance(traj.seg_start_idx.begin(), first_removed));

        traj.segs.resize(first_removed_idx);
        traj.seg_start_idx.resize(first_removed_idx);
        traj.seg_end_idx.resize(first_removed_idx);

        for (int i = first_tail_idx; i < N - 1; ++i) {
            const double duration = prefix[i + 1] - prefix[i];
            traj.push_seg(Seg::build(knots[i], knots[i + 1], duration, true), i, i + 1);
        }

        const double first_time =
            traj.seg_prefix_time.empty() ? prefix[first_tail_idx] : traj.seg_prefix_time.front();
        traj.rebuild_prefix(first_time);
    }

public:
    [[nodiscard]] inline GimbalState::State state_at(double t, const Traj& traj) const noexcept {
        if (traj.segs.empty())
            return {};
        if (t <= traj.seg_prefix_time[0])
            return traj.segs.front().eval(0.0);

        if (t >= traj.seg_prefix_time.back())
            return traj.segs.back().eval(traj.segs.back().T);

        const auto it =
            std::upper_bound(traj.seg_prefix_time.begin(), traj.seg_prefix_time.end(), t);

        size_t i = std::distance(traj.seg_prefix_time.begin(), it) - 1;
        i = std::min(i, traj.segs.size() - 1);

        const double t0 = traj.seg_prefix_time[i];
        return traj.segs[i].eval(t - t0);
    }
    [[nodiscard]] inline GimbalState state_at(double t) const noexcept {
        GimbalState::State yaw = state_at(t, yaw_traj);
        GimbalState::State pitch = state_at(t, pitch_traj);
        return GimbalState(yaw, pitch);
    }
};
class TinyMpcAxisSolver {
public:
    struct Params {
        float q_pos { 9e6f };
        float q_vel { 0.0f };
        float r { 1.0f };
        float max_acc { 50.0f };
        float state_min { -std::numeric_limits<float>::infinity() };
        float state_max { std::numeric_limits<float>::infinity() };
        int horizon { 100 };
        double dt;
        int max_iter;
    } params_;
    void reset() {
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int mpc_horizon = make_even(params_.horizon) + 1;
        auto dt = params_.dt;
        Eigen::MatrixXd A { { 1, dt }, { 0, 1 } };
        Eigen::MatrixXd B { { 0 }, { dt } };
        Eigen::VectorXd f { { 0, 0 } };
        Eigen::Matrix<double, 2, 1> Q { params_.q_pos, params_.q_vel };
        Eigen::Matrix<double, 1, 1> R { params_.r };
        tiny_setup(&_solver, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, mpc_horizon, 0);
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, mpc_horizon, params_.state_min);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, mpc_horizon, params_.state_max);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, mpc_horizon - 1, -params_.max_acc);
        Eigen::MatrixXd u_max_pitch =
            Eigen::MatrixXd::Constant(1, mpc_horizon - 1, params_.max_acc);
        tiny_set_bound_constraints(_solver, x_min, x_max, u_min, u_max_pitch);
        _solver->settings->max_iter = params_.max_iter;
    }
    bool solve(
        const Trajectory<GimbalState::State, double>& ref_traj,
        Trajectory<GimbalState::State, double>& control_traj,
        double t_current
    ) {
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int mpc_horizon = make_even(params_.horizon) + 1;
        const int half_horizon = make_even(params_.horizon) / 2;
        const auto trajVecToEigen = [&](const Trajectory<GimbalState::State, double>& traj) {
            Eigen::Matrix<double, 2, Eigen::Dynamic> mat(2, mpc_horizon);
            for (int k = 0; k < mpc_horizon; ++k) {
                int i = k - half_horizon;
                double t_add = i * params_.dt;
                double t = t_current + t_add;
                auto state = traj.Trajectory::state_at(t);
                mat(0, k) = state.p;
                mat(1, k) = state.v;
            }

            return mat;
        };
        auto traj_eigen = trajVecToEigen(ref_traj);
        Eigen::VectorXd x0(2);
        x0 << traj_eigen(0, 0), traj_eigen(1, 0);
        tiny_set_x0(_solver, x0);
        _solver->work->Xref = traj_eigen.block(0, 0, 2, mpc_horizon);
        tiny_solve(_solver);
        if (!_solver->work->status)
            return false;
        control_traj.clear();
        control_traj.reserve(mpc_horizon);
        for (int k = 0; k < mpc_horizon; ++k) {
            GimbalState::State tp;
            tp.p = _solver->work->x(0, k);
            tp.v = _solver->work->x(1, k);
            if (k == mpc_horizon - 1) {
                tp.a = _solver->work->u(0, k - 1);
            } else {
                tp.a = _solver->work->u(0, k);
            }
            int i = k - half_horizon;
            double t_add = i * params_.dt;
            double t = t_current + t_add;
            control_traj.push_back(tp, t);
        }
        return true;
    }

    TinySolver* _solver;
};
class TinyMpcTrajectory {
public:
    using Ptr = std::unique_ptr<TinyMpcTrajectory>;
    TinyMpcAxisSolver yaw_solver_;
    TinyMpcAxisSolver pitch_solver_;
    Trajectory<GimbalState::State, double> yaw_control_traj_;
    Trajectory<GimbalState::State, double> pitch_control_traj_;
    struct Params {
        double yaw_max_acc { 50.0f };
        double pitch_max_acc { 50.0f };
        int horizon { 100 };
        double dt;
        int max_iter;
    };
    TinyMpcTrajectory(const Params& p) {
        yaw_solver_.params_.dt = p.dt;
        pitch_solver_.params_.dt = p.dt;
        yaw_solver_.params_.horizon = p.horizon;
        pitch_solver_.params_.horizon = p.horizon;
        yaw_solver_.params_.max_iter = p.max_iter;
        pitch_solver_.params_.max_iter = p.max_iter;
        yaw_solver_.params_.max_acc = p.yaw_max_acc;
        pitch_solver_.params_.max_acc = p.pitch_max_acc;
        yaw_solver_.reset();
        pitch_solver_.reset();
    }
    static Ptr create(const Params& p) {
        return std::make_unique<TinyMpcTrajectory>(p);
    }
    void unwrap_states(std::vector<GimbalState>& s) const noexcept {
        if (s.size() < 2)
            return;
        for (size_t i = 1; i < s.size(); ++i) {
            s[i].yaw_state.p = angles::unwrap_angle(s[i - 1].yaw_state.p, s[i].yaw_state.p);
            s[i].pitch_state.p = angles::unwrap_angle(s[i - 1].pitch_state.p, s[i].pitch_state.p);
        }
    }
    bool solve(Trajectory<GimbalState, double>& ref_traj, double t_current) {
        unwrap_states(ref_traj.get_cp_vec());
        Trajectory<GimbalState::State, double> yaw_ref_traj;
        Trajectory<GimbalState::State, double> pitch_ref_traj;
        const auto& yp_cp = ref_traj.get_cp_vec();
        const auto& yp_prefix = ref_traj.get_prefix();
        for (int i = 0; i < yp_cp.size(); ++i) {
            yaw_ref_traj.push_back(yp_cp[i].yaw_state, yp_prefix[i]);
            pitch_ref_traj.push_back(yp_cp[i].pitch_state, yp_prefix[i]);
        }
        auto compute_va = [&](Trajectory<GimbalState::State, double>& j) {
            auto& s = j.get_cp_vec();
            const auto& prefix = j.get_prefix();
            s.front().v = s.back().v = 0.0;
            s.front().a = s.back().a = 0.0;

            for (size_t i = 1; i + 1 < s.size(); ++i) {
                const double dt0 = prefix[i] - prefix[i - 1];
                const double dt1 = prefix[i + 1] - prefix[i];
                const double denom = dt0 + dt1;
                const double w0 = dt1 / denom;
                const double w1 = dt0 / denom;

                s[i].v = w0 * (s[i].p - s[i - 1].p) / dt0 + w1 * (s[i + 1].p - s[i].p) / dt1;

                s[i].a = 2.0 * ((s[i + 1].p - s[i].p) / dt1 - (s[i].p - s[i - 1].p) / dt0) / denom;
            }
        };
        bool yaw_ok = true;
        bool pitch_ok = true;
        tbb::parallel_invoke(
            [&]() {
                compute_va(yaw_ref_traj);
                yaw_ok = yaw_solver_.solve(yaw_ref_traj, yaw_control_traj_, t_current);
            },
            [&]() {
                compute_va(pitch_ref_traj);
                pitch_ok = pitch_solver_.solve(pitch_ref_traj, pitch_control_traj_, t_current);
            }
        );
        return yaw_ok && pitch_ok;
    };
    [[nodiscard]] inline GimbalState state_at(double t) const noexcept {
        GimbalState::State yaw = yaw_control_traj_.state_at(t);
        GimbalState::State pitch = pitch_control_traj_.state_at(t);
        return GimbalState(yaw, pitch);
    }
};

class DualSmallMpcTrajectory {
public:
    using Ptr = std::unique_ptr<DualSmallMpcTrajectory>;
    Trajectory<GimbalState> control_traj_;
    talos::DualSmallMpcSolver::Ptr solver_;
    struct Params {
        float yaw_max_acc { 50.0f };
        float pitch_max_acc { 50.0f };
        int horizon { 100 };
        double dt;
        int max_iter;
    } params_;
    DualSmallMpcTrajectory(const Params& p) {
        params_ = p;
        auto yaw = talos::DualSmallMpcSolver::AxisConfig {
            .max_acc = p.yaw_max_acc,
        };
        auto pitch = talos::DualSmallMpcSolver::AxisConfig {
            .max_acc = p.pitch_max_acc,
        };
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int mpc_horizon = make_even(p.horizon) + 1;
        solver_ = talos::DualSmallMpcSolver::create(p.dt, mpc_horizon, 1.0f, yaw, pitch);
    }
    static Ptr create(const Params& p) {
        return std::make_unique<DualSmallMpcTrajectory>(p);
    }
    void unwrap_states(std::vector<GimbalState>& s) const noexcept {
        if (s.size() < 2)
            return;
        for (size_t i = 1; i < s.size(); ++i) {
            s[i].yaw_state.p = angles::unwrap_angle(s[i - 1].yaw_state.p, s[i].yaw_state.p);
            s[i].pitch_state.p = angles::unwrap_angle(s[i - 1].pitch_state.p, s[i].pitch_state.p);
        }
    }
    bool solve(Trajectory<GimbalState, double>& ref_traj, double t_current) {
        unwrap_states(ref_traj.get_cp_vec());
        auto compute_va = [&]() {
            auto& s = ref_traj.get_cp_vec();
            const auto& prefix = ref_traj.get_prefix();
            s.front().yaw_state.v = s.back().yaw_state.v = 0.0;
            s.front().pitch_state.v = s.back().pitch_state.v = 0.0;
            s.front().yaw_state.a = s.back().yaw_state.a = 0.0;
            s.front().pitch_state.a = s.back().pitch_state.a = 0.0;

            for (size_t i = 1; i + 1 < s.size(); ++i) {
                const double dt0 = prefix[i] - prefix[i - 1];
                const double dt1 = prefix[i + 1] - prefix[i];
                const double denom = dt0 + dt1;
                const double w0 = dt1 / denom;
                const double w1 = dt0 / denom;

                s[i].yaw_state.v = w0 * (s[i].yaw_state.p - s[i - 1].yaw_state.p) / dt0
                    + w1 * (s[i + 1].yaw_state.p - s[i].yaw_state.p) / dt1;
                s[i].pitch_state.v = w0 * (s[i].pitch_state.p - s[i - 1].pitch_state.p) / dt0
                    + w1 * (s[i + 1].pitch_state.p - s[i].pitch_state.p) / dt1;

                s[i].yaw_state.a = 2.0
                    * ((s[i + 1].yaw_state.p - s[i].yaw_state.p) / dt1
                       - (s[i].yaw_state.p - s[i - 1].yaw_state.p) / dt0)
                    / denom;
                s[i].pitch_state.a = 2.0
                    * ((s[i + 1].pitch_state.p - s[i].pitch_state.p) / dt1
                       - (s[i].pitch_state.p - s[i - 1].pitch_state.p) / dt0)
                    / denom;
            }
        };
        compute_va();
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int mpc_horizon = make_even(params_.horizon) + 1;
        const int half_horizon = make_even(params_.horizon) / 2;
        for (int k = 0; k < mpc_horizon; ++k) {
            int i = k - half_horizon;
            double t_add = i * params_.dt;
            double t = t_current + t_add;
            auto state = ref_traj.Trajectory::state_at(t);
            solver_->set_ref_col(
                k,
                state.yaw_state.p,
                state.yaw_state.v,
                state.pitch_state.p,
                state.pitch_state.v
            );
            if (k == 0) {
                solver_->set_x0(
                    state.yaw_state.p,
                    state.yaw_state.v,
                    state.pitch_state.p,
                    state.pitch_state.v
                );
            }
        }
        solver_->solve();
        // if (!solver_->solve()) {
        //     return false;
        // }
        control_traj_.clear();
        control_traj_.reserve(mpc_horizon);
        for (int k = 0; k < mpc_horizon; ++k) {
            GimbalState tp;
            tp.yaw_state.p = solver_->state(talos::DualSmallMpcSolver::kYawAxis, 0, k);
            tp.yaw_state.v = solver_->state(talos::DualSmallMpcSolver::kYawAxis, 1, k);
            tp.pitch_state.p = solver_->state(talos::DualSmallMpcSolver::kPitchAxis, 0, k);
            tp.pitch_state.v = solver_->state(talos::DualSmallMpcSolver::kPitchAxis, 1, k);

            if (k == mpc_horizon - 1) {
                tp.yaw_state.a = solver_->input(talos::DualSmallMpcSolver::kYawAxis, k - 1);
                tp.pitch_state.a = solver_->input(talos::DualSmallMpcSolver::kPitchAxis, k - 1);
            } else {
                tp.yaw_state.a = solver_->input(talos::DualSmallMpcSolver::kYawAxis, k);
                tp.pitch_state.a = solver_->input(talos::DualSmallMpcSolver::kPitchAxis, k);
            }

            int i = k - half_horizon;
            double t_add = i * params_.dt;
            double t = t_current + t_add;
            control_traj_.push_back(tp, t);
        }
        return true;
    };
    [[nodiscard]] inline GimbalState state_at(double t) const noexcept {
        return control_traj_.state_at(t);
    }
};
} // namespace awakening::dta_utils
