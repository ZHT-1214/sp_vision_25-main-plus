#pragma once
#include "angles.h"
#include "utils/common/type_common.hpp"
#include "utils/utils.hpp"
#include <array>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <vector>
namespace awakening::auto_buff {
constexpr double RUNE_FAN_TARGET_BOX_DIS_HALF = 0.15 / 2.0;
constexpr double RUNE_FAN_TARGET_R = 0.115;
constexpr double RUNE_R2_FAN_TARGET_CENTER = 0.70;
constexpr double FUCK = 0.015;
constexpr int FAN_NUM = 5;
enum class RuneColor : int {
    RED = 0,
    BLUE = 1,
    NONE = -1,
};

struct RuneFanBladeWithR {
    enum PointsIndex { TOP, LEFT, BOTTOM, RIGHT, CENTER, R, N };
    template<typename PointT>
    struct Point3DRZERO {
        inline static std::vector<PointT> build() {
            return {
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER + RUNE_FAN_TARGET_R), // 上
                PointT(0, RUNE_FAN_TARGET_R, RUNE_R2_FAN_TARGET_CENTER), // 左
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER - RUNE_FAN_TARGET_R), // 下
                PointT(0, -RUNE_FAN_TARGET_R, RUNE_R2_FAN_TARGET_CENTER), // 右
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER), // 中
                PointT(0, 0, 0),
            };
        }
        inline static std::vector<PointT> build_no_r() {
            return {
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER + RUNE_FAN_TARGET_R), // 上
                PointT(0, RUNE_FAN_TARGET_R, RUNE_R2_FAN_TARGET_CENTER), // 左
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER - RUNE_FAN_TARGET_R), // 下
                PointT(0, -RUNE_FAN_TARGET_R, RUNE_R2_FAN_TARGET_CENTER), // 右
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER), // 中
            };
        }
    };
    template<typename PointT>
    struct Point3DTargetCenterZERO {
        inline static std::vector<PointT> build() {
            return {
                PointT(0, 0, +RUNE_FAN_TARGET_R), // 上
                PointT(0, RUNE_FAN_TARGET_R, 0), // 左
                PointT(0, 0, -RUNE_FAN_TARGET_R), // 下
                PointT(0, -RUNE_FAN_TARGET_R, 0), // 右
                PointT(0, 0, 0), // 中
                PointT(0, 0, -RUNE_R2_FAN_TARGET_CENTER),
            };
        }
        inline static std::vector<PointT> build_no_r() {
            return {
                PointT(0, 0, +RUNE_FAN_TARGET_R), // 上
                PointT(0, RUNE_FAN_TARGET_R, 0), // 左
                PointT(0, 0, -RUNE_FAN_TARGET_R), // 下
                PointT(0, -RUNE_FAN_TARGET_R, 0), // 右
                PointT(0, 0, 0), // 中
            };
        }
    };

    std::vector<cv::Point2f> points;
    std::vector<std::vector<cv::Point2f>> tmp_points;
    ISO3 pose;
    cv::Rect2f bbox;
    RuneColor color = RuneColor::NONE;
    double confidence = 0;
    void draw(cv::Mat& img) const {
        for (int i = 0; i < points.size(); ++i) {
            cv::circle(img, points[i], 5, cv::Scalar(0, 255, 0), cv::FILLED);
        }
    }
    void add_offset(const cv::Point2f& offset) {
        for (auto& point: points) {
            point += offset;
        }
        bbox += offset;
    }
    void transform(const Eigen::Matrix<float, 3, 3>& transform_matrix) noexcept {
        for (auto& point: points) {
            point = utils::transform_point2D(transform_matrix, point);
        }
        bbox = utils::transform_rect(transform_matrix, bbox);
    }
};
struct RuneR {
    cv::RotatedRect rr;
    bool laji = true;
    RuneColor color = RuneColor::NONE;
    void add_offset(const cv::Point2f& offset) {
        rr.center += offset;
    }
    void draw(cv::Mat& img) const {
        cv::Point2f vertices[4];
        rr.points(vertices);
        for (int i = 0; i < 4; ++i) {
            cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(img, rr.center, 3, cv::Scalar(0, 0, 255), cv::FILLED);
    }
};
struct RuneFlowingLight {
    cv::RotatedRect rr;
    void add_offset(const cv::Point2f& offset) {
        rr.center += offset;
    }
    void draw(cv::Mat& img) const {
        cv::Point2f vertices[4];
        rr.points(vertices);
        for (int i = 0; i < 4; ++i) {
            cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
    }
};

struct RuneFanTarget {
    std::array<cv::Point2f, 4> corners;
    std::vector<cv::Point2f> key_points;
    cv::Point2f center;
    RuneColor color = RuneColor::NONE;
    cv::RotatedRect rr;
    ISO3 pose;
    enum PointsIndex { LT, LB, RB, RT, CENTER, N };
    template<typename PointT>
    struct Point3DRZERO {
        inline static std::vector<PointT> build_no_r() {
            return {
                PointT(
                    0,
                    RUNE_FAN_TARGET_BOX_DIS_HALF,
                    RUNE_R2_FAN_TARGET_CENTER + RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 左上
                PointT(
                    0,
                    RUNE_FAN_TARGET_BOX_DIS_HALF,
                    RUNE_R2_FAN_TARGET_CENTER - RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 左下
                PointT(
                    0,
                    -RUNE_FAN_TARGET_BOX_DIS_HALF,
                    RUNE_R2_FAN_TARGET_CENTER - RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 右下
                PointT(
                    0,
                    -RUNE_FAN_TARGET_BOX_DIS_HALF,
                    RUNE_R2_FAN_TARGET_CENTER + RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 右上
                PointT(0, 0, RUNE_R2_FAN_TARGET_CENTER), // 中
            };
        }
    };
    template<typename PointT>
    struct Point3DTargetCenterZERO {
        inline static std::vector<PointT> build_no_r() {
            return {
                PointT(
                    0,
                    RUNE_FAN_TARGET_BOX_DIS_HALF,
                    RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 左上
                PointT(
                    0,
                    RUNE_FAN_TARGET_BOX_DIS_HALF,
                    -RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 左下
                PointT(
                    0,
                    -RUNE_FAN_TARGET_BOX_DIS_HALF,
                    -RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 右下
                PointT(
                    0,
                    -RUNE_FAN_TARGET_BOX_DIS_HALF,
                    +RUNE_FAN_TARGET_BOX_DIS_HALF
                ), // 右上
                PointT(0, 0, 0), // 中
            };
        }
    };
    void add_offset(const cv::Point2f& offset) {
        center += offset;
        for (auto& corner: corners) {
            corner += offset;
        }
    }

    void draw(cv::Mat& img) const {
        for (int i = 0; i < 4; ++i) {
            cv::circle(img, corners[i], 5, cv::Scalar(255, 0, 255), cv::FILLED);
        }
        cv::circle(img, center, 7, cv::Scalar(0, 0, 255), cv::FILLED);
    }
    void sort_corners(const cv::Point2f& r) {
        if (corners.size() != 4)
            return;

        cv::Point2f down_vec = r - center;
        float norm = std::sqrt(down_vec.x * down_vec.x + down_vec.y * down_vec.y);
        float angle_ref = std::atan2(down_vec.y, down_vec.x);
        struct Node {
            float ang;
            cv::Point2f p;
        };
        std::vector<Node> arr;
        arr.reserve(4);

        for (auto& p: corners) {
            cv::Point2f v = p - center;
            float ang = std::atan2(v.y, v.x) - angle_ref;

            ang = angles::normalize_angle(ang);

            arr.push_back({ ang, p });
        }

        std::sort(arr.begin(), arr.end(), [](const Node& a, const Node& b) {
            return a.ang < b.ang;
        });

        cv::Point2f lu(0, 0), ru(0, 0), rd(0, 0), ld(0, 0);
        bool has_lu = false, has_ru = false, has_rd = false, has_ld = false;

        for (const auto& n: arr) {
            float a = n.ang;

            if (a > CV_PI / 2 && a <= CV_PI) {
                lu = n.p;
                has_lu = true;
            } else if (a > 0 && a <= CV_PI / 2) {
                ru = n.p;
                has_ru = true;
            } else if (a > -CV_PI / 2 && a <= 0) {
                rd = n.p;
                has_rd = true;
            } else { // a > -CV_PI && a <= -CV_PI/2
                ld = n.p;
                has_ld = true;
            }
        }

        std::array<cv::Point2f, 4> ordered;

        if (has_lu && has_ru && has_rd && has_ld) {
            ordered[0] = lu;
            ordered[1] = ru;
            ordered[2] = rd;
            ordered[3] = ld;
            key_points.assign(ordered.begin(), ordered.end());
            key_points.push_back(center);
            return;
        }

        float target = 3.0f * CV_PI / 4.0f; // 135°
        int best_idx = 0;
        float best_diff = std::numeric_limits<float>::max();
        for (int i = 0; i < (int)arr.size(); ++i) {
            float d = std::fabs(angles::shortest_angular_distance(target, arr[i].ang));
            if (d < best_diff) {
                best_diff = d;
                best_idx = i;
            }
        }

        for (int i = 0; i < 4; ++i) {
            int idx = (best_idx + i) % 4;
            ordered[i] = arr[idx].p;
        }

        key_points.assign(ordered.begin(), ordered.end());
        key_points.push_back(center);
    }
};

struct RuneDetection {
    TimePoint timestamp;
    int id = -1;
    int frame_id = -1;

    std::vector<RuneFanBladeWithR> fan_blades;
    std::vector<RuneR> rune_rs;
    std::vector<RuneFanTarget> fan_targets;
    std::vector<RuneFlowingLight> rune_flowing_lights;
    void draw(cv::Mat& img) const {
        for (const auto& fan_blade: fan_blades) {
            fan_blade.draw(img);
        }
        for (const auto& rune_r: rune_rs) {
            rune_r.draw(img);
        }
        for (const auto& fan_target: fan_targets) {
            fan_target.draw(img);
        }
        for (const auto& rune_flowing_light: rune_flowing_lights) {
            rune_flowing_light.draw(img);
        }
    }
};
} // namespace awakening::auto_buff