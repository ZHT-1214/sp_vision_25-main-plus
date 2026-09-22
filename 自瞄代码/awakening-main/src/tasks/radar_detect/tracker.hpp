#pragma once
#include "Hungarian/Hungarian.h"
#include "tasks/radar_detect/type.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <opencv2/core/types.hpp>
#include <unordered_map>
#include <utility>
#include <vector>
#include <yaml-cpp/node/node.h>

namespace awakening::radar_detect {

struct TrackerConfig {
    double max_match_cost;
    int track_threshold;
    double lost_time;
    double v_damping;
    double iou_thresh;

    PointTarget::PointTargetConfig p_cfg;

    double w_dist;
    double w_iou;
    double w_id;

    void load(const YAML::Node& config) {
        max_match_cost = config["max_match_cost"].as<double>();
        lost_time = config["lost_time"].as<double>();
        track_threshold = config["track_threshold"].as<int>();
        iou_thresh = config["iou_thresh"].as<double>();

        p_cfg.load(config["p_target"]);

        w_dist = config["w_dist"].as<double>();
        w_iou = config["w_iou"].as<double>();
        w_id = config["w_id"].as<double>();
    }
};

enum class TargetState { TENTATIVE, CONFIRMED, LOST };

struct Target {
    Target() {}

    Target(int id, const Car& car, const TrackerConfig& track_cfg) {
        track_id = id;

        tracker_config = track_cfg;

        target_state = TargetState::TENTATIVE;

        uwb_state = PointTarget(track_cfg.p_cfg, car.point_in_uwb, car.timestamp);

        last_update = car.timestamp;

        bbox = car.bbox;

        bot_id_history.push_back(car.get_car_class());

        fin_class = car.get_car_class();
    }

    void predict_ekf(const TimePoint& t) {
        uwb_state.predict_ekf(t);
    }

    void update(const Car& car) {
        bbox = car.bbox;

        last_update = car.timestamp;

        uwb_state.update(car.point_in_uwb, car.timestamp);

        hit_count++;

        miss_count = 0;

        if (car.get_car_class() != CarClass::UNKNOWN) {
            bot_id_history.push_back(car.get_car_class());
        }

        if (bot_id_history.size() > 100) {
            bot_id_history.pop_front();
        }

        std::unordered_map<int, int> freq;

        for (auto id: bot_id_history) {
            freq[std::to_underlying(id)]++;
        }

        int max_count = 0;

        int most_common_bot_id = std::to_underlying(fin_class);

        for (const auto& kv: freq) {
            if (kv.second > max_count) {
                max_count = kv.second;

                most_common_bot_id = kv.first;
            }
        }

        fin_class = CarClass(most_common_bot_id);
    }

    int track_id = -1;

    int hit_count = 0;

    int miss_count = 0;

    TrackerConfig tracker_config;

    CarClass fin_class = CarClass::UNKNOWN;

    std::deque<CarClass> bot_id_history;

    TargetState target_state = TargetState::TENTATIVE;

    cv::Rect2f bbox;

    PointTarget uwb_state;

    TimePoint last_update;
};

class Tracker {
public:
    Tracker(const YAML::Node& config) {
        tracker_config_.load(config);
    }

    float compute_iou(const cv::Rect2f& a, const cv::Rect2f& b) const noexcept {
        float inter_left = std::max(a.x, b.x);

        float inter_right = std::min(a.x + a.width, b.x + b.width);

        float inter_top = std::max(a.y, b.y);

        float inter_bottom = std::min(a.y + a.height, b.y + b.height);

        float inter_width = inter_right - inter_left;

        float inter_height = inter_bottom - inter_top;

        if (inter_width <= 0 || inter_height <= 0) {
            return 0.0f;
        }

        float inter_area = inter_width * inter_height;

        float area_a = a.width * a.height;

        float area_b = b.width * b.height;

        return inter_area / (area_a + area_b - inter_area);
    }

    void nms(std::vector<Car>& cars) const noexcept {
        std::sort(cars.begin(), cars.end(), [](const Car& a, const Car& b) {
            return a.confidence > b.confidence;
        });

        std::vector<bool> suppressed(cars.size(), false);

        for (size_t i = 0; i < cars.size(); ++i) {
            if (suppressed[i]) {
                continue;
            }

            for (size_t j = i + 1; j < cars.size(); ++j) {
                if (suppressed[j]) {
                    continue;
                }

                if (compute_iou(cars[i].bbox, cars[j].bbox) > tracker_config_.iou_thresh) {
                    suppressed[j] = true;
                }
            }
        }

        std::vector<Car> filtered;

        for (size_t i = 0; i < cars.size(); ++i) {
            if (!suppressed[i]) {
                filtered.push_back(cars[i]);
            }
        }

        cars = std::move(filtered);
    }

    double compute_cost(const Target& target, const Car& car) const noexcept {
        double dist = (target.uwb_state.state.pos() - car.point_in_uwb).norm();

        double iou = compute_iou(target.bbox, car.bbox);

        double bot_id_penalty =
            (target.fin_class != CarClass::UNKNOWN && target.fin_class != car.get_car_class())
            ? 10.0
            : 0.0;

        double W_DIST = tracker_config_.w_dist;

        double W_IOU = tracker_config_.w_iou;

        double W_BOTID = tracker_config_.w_id;

        double cost = W_DIST * dist + W_IOU * (1.0 - iou) + W_BOTID * bot_id_penalty;

        return cost;
    }

    void match(
        const std::vector<int>& target_ids,
        const std::vector<const Car*>& cars,
        std::vector<std::pair<int, int>>& matches,
        std::vector<int>& unmatched_targets,
        std::vector<int>& unmatched_cars
    ) noexcept {
        if (target_ids.empty() || cars.empty()) {
            matches.clear();

            unmatched_targets.clear();

            unmatched_cars.clear();

            for (int i = 0; i < static_cast<int>(target_ids.size()); ++i) {
                unmatched_targets.push_back(i);
            }

            for (int j = 0; j < static_cast<int>(cars.size()); ++j) {
                unmatched_cars.push_back(j);
            }

            return;
        }

        int N = static_cast<int>(target_ids.size());

        int M = static_cast<int>(cars.size());

        int L = std::max(N, M);

        std::vector<std::vector<double>> cost_matrix(L, std::vector<double>(L, 1e6));

        for (int i = 0; i < N; ++i) {
            const auto& target = targets_.at(target_ids[i]);

            for (int j = 0; j < M; ++j) {
                double cost = compute_cost(target, *cars[j]);

                if (cost < 0.0) {
                    cost = 0.0;
                }

                if (cost < tracker_config_.max_match_cost) {
                    cost_matrix[i][j] = cost;
                }
            }
        }

        HungarianAlgorithm HungAlgo;

        std::vector<int> assignment;

        [[maybe_unused]] double cost_sum = HungAlgo.Solve(cost_matrix, assignment);

        matches.clear();

        unmatched_targets.clear();

        unmatched_cars.clear();

        std::vector<bool> det_assigned(M, false);

        for (int i = 0; i < N; ++i) {
            if (i >= static_cast<int>(assignment.size())) {
                unmatched_targets.push_back(i);
                continue;
            }

            int j = assignment[i];

            if (j >= 0 && j < M) {
                if (cost_matrix[i][j] < tracker_config_.max_match_cost) {
                    matches.emplace_back(i, j);

                    det_assigned[j] = true;
                } else {
                    unmatched_targets.push_back(i);
                }
            } else {
                unmatched_targets.push_back(i);
            }
        }

        for (int j = 0; j < M; ++j) {
            if (!det_assigned[j]) {
                unmatched_cars.push_back(j);
            }
        }
    }

    void update(const std::vector<Car>& input_cars, const TimePoint& t) noexcept {
        std::vector<Car> cars = input_cars;

        nms(cars);

        std::vector<int> target_ids;

        for (auto& [id, target]: targets_) {
            if (target.target_state == TargetState::LOST) {
                continue;
            }

            target.predict_ekf(t);

            target_ids.push_back(id);
        }

        std::vector<const Car*> p_cars;

        for (auto& car: cars) {
            p_cars.push_back(&car);
        }

        double dt = std::chrono::duration<double>(t - last_track_).count();

        dt = std::clamp(dt, 1e-3, 0.1);

        last_track_ = t;

        lost_thres_ = std::clamp(static_cast<int>(tracker_config_.lost_time / dt), 1, 100);

        std::vector<std::pair<int, int>> matches;

        std::vector<int> unmatched_targets;

        std::vector<int> unmatched_cars;

        match(target_ids, p_cars, matches, unmatched_targets, unmatched_cars);

        for (const auto& [t_idx, c_idx]: matches) {
            int target_id = target_ids[t_idx];

            Target& target = targets_.at(target_id);

            const Car& car = *p_cars[c_idx];

            target.update(car);

            if (target.target_state == TargetState::TENTATIVE
                && target.hit_count >= tracker_config_.track_threshold)
            {
                target.target_state = TargetState::CONFIRMED;
            } else if (target.target_state == TargetState::LOST) {
                target.target_state = TargetState::CONFIRMED;

                target.miss_count = 0;
            }
        }

        for (int idx: unmatched_targets) {
            int target_id = target_ids[idx];

            Target& target = targets_.at(target_id);

            target.miss_count++;

            auto vel = target.uwb_state.state.vel();

            vel *= tracker_config_.v_damping;

            target.uwb_state.state.set_vel(vel);

            target.uwb_state.set_ekf_state(target.uwb_state.state);

            if (target.miss_count > lost_thres_) {
                target.target_state = TargetState::LOST;
            }
        }

        for (int idx: unmatched_cars) {
            const Car& car = *p_cars[idx];

            Target new_target(next_track_id_++, car, tracker_config_);

            targets_[new_target.track_id] = std::move(new_target);
        }

        std::vector<int> to_erase;

        for (auto& [id, target]: targets_) {
            if (target.target_state == TargetState::LOST && target.miss_count > 2 * lost_thres_) {
                to_erase.push_back(id);
            }
        }

        for (int id: to_erase) {
            targets_.erase(id);
        }
    }

    std::vector<Target> get_targets() const noexcept {
        std::vector<Target> result;

        result.reserve(targets_.size());

        for (const auto& [id, target]: targets_) {
            result.push_back(target);
        }

        return result;
    }

public:
    int next_track_id_ = 0;

    int lost_thres_ = 5;

    TrackerConfig tracker_config_;

    std::unordered_map<int, Target> targets_;

    TimePoint last_track_ = Clock::now();
};

} // namespace awakening::radar_detect