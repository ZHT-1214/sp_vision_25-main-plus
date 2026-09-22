#pragma once
#include "utils/common/type_common.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <shared_mutex>

namespace awakening::utils {

struct TimedPose {
    TimePoint stamp;
    ISO3 pose;
};

class TimePoseBuffer {
public:
    explicit TimePoseBuffer(size_t max_size = 1024): max_size_(max_size) {}

    void push(const TimePoint& t, const ISO3& pose) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        if (max_size_ == 0)
            return;
        if (!buffer_.empty() && t < buffer_.back().stamp) {
            std::cout << "Time is out of order!" << std::endl;
            return;
        }

        buffer_.push_back({ t, pose });
        while (buffer_.size() > max_size_)
            buffer_.pop_front();
    }

    ISO3 get(const TimePoint& t) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);

        if (buffer_.empty())
            return ISO3::Identity();
        if (buffer_.size() == 1)
            return buffer_.front().pose;

        auto it = std::lower_bound(
            buffer_.begin(),
            buffer_.end(),
            t,
            [](const TimedPose& p, const TimePoint& t) { return p.stamp < t; }
        );

        if (it == buffer_.begin()) {
            // std::cout << "its begin" << std::endl;
            return it->pose;
        }

        if (it == buffer_.end()) {
            // std::cout << "its end" << std::endl;
            return extrapolate(t);
        }

        const auto& p1 = *(it - 1);
        const auto& p2 = *it;

        const double dt = seconds(p2.stamp - p1.stamp);
        if (std::abs(dt) < kMinDt)
            return p2.pose;

        double r = std::clamp(seconds(t - p1.stamp) / dt, 0.0, 1.0);

        Vec3 trans = (1 - r) * p1.pose.translation() + r * p2.pose.translation();
        Quaternion q1(p1.pose.rotation());
        Quaternion q2(p2.pose.rotation());
        normalize_pair(q1, q2);
        Quaternion q = q1.slerp(r, q2).normalized();

        ISO3 T = ISO3::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = trans;
        return T;
    }

private:
    static constexpr double kMinDt = 1e-6;
    static constexpr double kMinOmega = 1e-9;

    template<typename Duration>
    static double seconds(const Duration& duration) {
        return std::chrono::duration<double>(duration).count();
    }

    static void normalize_pair(Quaternion& q1, Quaternion& q2) {
        q1.normalize();
        q2.normalize();
        if (q1.dot(q2) < 0.0)
            q2.coeffs() *= -1.0;
    }

    ISO3 extrapolate(const TimePoint& t) const {
        if (buffer_.size() < 2)
            return buffer_.back().pose;

        const auto& p1 = buffer_[buffer_.size() - 2];
        const auto& p2 = buffer_.back();

        double dt = seconds(p2.stamp - p1.stamp);
        if (dt < kMinDt)
            return p2.pose;

        double dt_future = seconds(t - p2.stamp);

        Vec3 v = (p2.pose.translation() - p1.pose.translation()) / dt;
        Vec3 trans = p2.pose.translation() + v * dt_future;

        Quaternion q1(p1.pose.rotation());
        Quaternion q2(p2.pose.rotation());
        normalize_pair(q1, q2);
        Quaternion dq = q1.inverse() * q2;
        dq.normalize();

        AngleAxis aa(dq);
        Vec3 omega = aa.axis() * aa.angle() / dt;

        double omega_norm = omega.norm();
        double angle = omega_norm * dt_future;
        Vec3 axis = Vec3::UnitX();
        if (omega_norm > kMinOmega)
            axis = omega / omega_norm;

        Quaternion q_future = (q2 * Quaternion(AngleAxis(angle, axis))).normalized();

        ISO3 T = ISO3::Identity();
        T.linear() = q_future.toRotationMatrix();
        T.translation() = trans;
        return T;
    }

private:
    mutable std::shared_mutex mutex_;
    std::deque<TimedPose> buffer_;
    size_t max_size_;
};

} // namespace awakening::utils
