#pragma once
#include "tasks/base/common.hpp"
#include "utils/utils.hpp"
#include <array>
#include <cstddef>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>
namespace awakening::auto_aim {
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;
enum class ArmorClass : int { SENTRY = 0, NO1, NO2, NO3, NO4, NO5, OUTPOST, BASE, UNKNOWN };
enum class ArmorType : int { SimpleSmall, Large };
template<ArmorType T>
struct ArmorTypeTraits; // declare
template<>
struct ArmorTypeTraits<ArmorType::SimpleSmall> {
    static constexpr double WIDTH = 133.0 / 1000.0;
    static constexpr double HEIGHT = 50.0 / 1000.0;
};
template<>
struct ArmorTypeTraits<ArmorType::Large> {
    static constexpr double WIDTH = 225.0 / 1000.0;
    static constexpr double HEIGHT = 50.0 / 1000.0;
};
template<typename PointT, ArmorType T>
struct ArmorKeyPoint3D {
    static constexpr double W = ArmorTypeTraits<T>::WIDTH;
    static constexpr double H = ArmorTypeTraits<T>::HEIGHT;
    inline static std::vector<PointT> build() {
        return {
            PointT(0, W / 2, H / 2), // 左上
            PointT(0, W / 2, -H / 2), // 左下
            PointT(0, -W / 2, -H / 2), // 右下
            PointT(0, -W / 2, H / 2), // 右上
            // PointT(0, W / 2, 0),       PointT(0, -W / 2, 0),
        };
    }
    inline static std::vector<PointT> build_light(bool left) {
        return {
            PointT(0, left ? W / 2 : -W / 2, H / 2), // 左上或右上
            PointT(0, left ? W / 2 : -W / 2, -H / 2), // 左下或右下
        };
    }
};
enum ArmorKeyPointsIndex {
    LEFT_TOP,
    LEFT_BOTTOM,
    RIGHT_BOTTOM,
    RIGHT_TOP,
    // LEFT_MID,
    // RIGHT_MID,
    N
};
inline std::string string_by_armor_key_points_index(int index) {
    constexpr const char* details[] = { "left_top",  "left_bottom", "right_bottom",
                                        "right_top", "left_mid",    "right_mid" };
    return std::string(details[index]);
}

struct ArmorKeyPoints2D {
    using PointT = cv::Point2f;
    using I = ArmorKeyPointsIndex;

    inline void add_offset(const PointT& offset) noexcept {
        for (auto& p_opt: points) {
            if (p_opt) {
                *p_opt += offset;
            }
        }
        // compute_mid(I::LEFT_MID, I::LEFT_TOP, I::LEFT_BOTTOM);
        // compute_mid(I::RIGHT_MID, I::RIGHT_TOP, I::RIGHT_BOTTOM);
        full_points.reset();
        bbox.reset();
    }

    inline void transform(const Eigen::Matrix<float, 3, 3>& transform_matrix) noexcept {
        for (auto& p_opt: points) {
            if (p_opt) {
                *p_opt = utils::transform_point2D(transform_matrix, *p_opt);
            }
        }
        // compute_mid(I::LEFT_MID, I::LEFT_TOP, I::LEFT_BOTTOM);
        // compute_mid(I::RIGHT_MID, I::RIGHT_TOP, I::RIGHT_BOTTOM);
        full_points.reset();
        bbox.reset();
    }
    inline void compute_mid(I mid, I top, I bottom) {
        auto& mid_opt = points[std::to_underlying(mid)];
        mid_opt = (*points[std::to_underlying(top)] + *points[std::to_underlying(bottom)]) / 2.f;
    };

    inline std::array<PointT, std::to_underlying(I::N)>& landmarks() {
        if (!full_points) {
            // compute_mid(I::LEFT_MID, I::LEFT_TOP, I::LEFT_BOTTOM);
            // compute_mid(I::RIGHT_MID, I::RIGHT_TOP, I::RIGHT_BOTTOM);
            std::array<PointT, std::to_underlying(I::N)> tmp {};
            for (size_t i = 0; i < points.size(); ++i) {
                if (!points[i]) {
                    throw std::runtime_error("ArmorKeyPoints2D::points(): point not set");
                }
                tmp[i] = *points[i];
            }
            full_points = tmp;
        }

        return *full_points;
    }
    inline cv::Rect2f bounding_box() {
        if (!bbox.has_value()) {
            float min_x = std::numeric_limits<float>::max();
            float min_y = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float max_y = std::numeric_limits<float>::lowest();

            bool has_point = false;

            for (const auto& p: points) {
                if (!p.has_value()) {
                    continue;
                }
                const auto& pt = p.value();
                min_x = std::min(min_x, pt.x);
                min_y = std::min(min_y, pt.y);
                max_x = std::max(max_x, pt.x);
                max_y = std::max(max_y, pt.y);
                has_point = true;
            }
            if (!has_point) {
                throw std::runtime_error("No point in the contour");
            }
            bbox = cv::Rect2f { min_x, min_y, max_x - min_x, max_y - min_y };
        }

        return bbox.value();
    }

    std::array<std::optional<PointT>, std::to_underlying(I::N)> points {};

private:
    std::optional<std::array<PointT, std::to_underlying(I::N)>> full_points;
    std::optional<cv::Rect2f> bbox;
};
inline ArmorType armor_type_by_armor_class(ArmorClass armor_class) {
    if (armor_class == ArmorClass::NO1) {
        return ArmorType::Large;
    } else {
        return ArmorType::SimpleSmall;
    }
}
template<typename PointT>
inline std::vector<PointT> getArmorKeyPoints3D(ArmorClass armor_class) {
    auto armor_type = armor_type_by_armor_class(armor_class);
    if (armor_type == ArmorType::Large) {
        return ArmorKeyPoint3D<PointT, ArmorType::Large>::build();
    } else {
        return ArmorKeyPoint3D<PointT, ArmorType::SimpleSmall>::build();
    }
}

inline std::pair<double, double> getArmorWH(ArmorClass armor_class) {
    auto armor_type = armor_type_by_armor_class(armor_class);
    if (armor_type == ArmorType::Large) {
        return std::make_pair(
            ArmorKeyPoint3D<cv::Point3f, ArmorType::Large>::W,
            ArmorKeyPoint3D<cv::Point3f, ArmorType::Large>::H
        );
    } else {
        return std::make_pair(
            ArmorKeyPoint3D<cv::Point3f, ArmorType::SimpleSmall>::W,
            ArmorKeyPoint3D<cv::Point3f, ArmorType::SimpleSmall>::H
        );
    }
}
template<typename PointT>
inline std::vector<PointT> getArmorLightKeyPoints3D(ArmorClass armor_class, bool left) {
    auto armor_type = armor_type_by_armor_class(armor_class);
    if (armor_type == ArmorType::Large) {
        return ArmorKeyPoint3D<PointT, ArmorType::Large>::build_light(left);
    } else {
        return ArmorKeyPoint3D<PointT, ArmorType::SimpleSmall>::build_light(left);
    }
}
enum class ArmorColor : int { BLUE = 0, RED, NONE, PURPLE };

inline int armor_num_by_armor_class(const ArmorClass& armor_class) {
    if (armor_class == ArmorClass::OUTPOST) {
        return 3;
    } else if (armor_class == ArmorClass::BASE) {
        return 1;
    } else {
        return 4;
    }
}
inline std::string string_by_armor_color(ArmorColor armor_color) {
    constexpr const char* details[] = { "blue", "red", "none", "purple" };
    return std::string(details[std::to_underlying(armor_color)]);
}
inline cv::Scalar CV_color_by_armor_class(ArmorColor armor_color) {
    static cv::Scalar details[] = { cv::Scalar(255, 0, 0),
                                    cv::Scalar(0, 0, 255),
                                    cv::Scalar(255, 255, 255),
                                    cv::Scalar(255, 0, 255) };
    return details[std::to_underlying(armor_color)];
}
inline std::string string_by_armor_class(ArmorClass armor_class) {
    constexpr const char* details[] = { "sentry", "no1",     "no2",  "no3",    "no4",
                                        "no5",    "outpost", "base", "unknown" };
    return std::string(details[std::to_underlying(armor_class)]);
}
struct Light: public cv::RotatedRect {
    Light() = default;

    explicit Light(const std::vector<cv::Point>& contour, size_t id):
        cv::RotatedRect(cv::minAreaRect(contour)),
        id(id) {
        this->center = std::accumulate(
            contour.begin(),
            contour.end(),
            cv::Point2f(0, 0),
            [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                return a + cv::Point2f(b.x, b.y) / n;
            }
        );

        cv::Point2f p[4];
        this->points(p);

        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        axis = (top - bottom) / cv::norm(top - bottom);

        tilt_angle =
            std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) / CV_PI * 180.0f;
    }
    void add_offset(const cv::Point2f& offset) noexcept {
        this->center += offset;
        top += offset;
        bottom += offset;
        if (corrected) {
            corrected->first += offset;
            corrected->second += offset;
        }
    }
    inline void draw(cv::Mat& img) const noexcept {
        // cv::line(img, top, bottom, cv::Scalar(100, 255, 100), 2);
        cv::circle(img, top, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, bottom, 3, cv::Scalar(0, 255, 0), -1);
    }
    std::optional<std::pair<cv::Point2f, cv::Point2f>> corrected;
    cv::Point2f top, bottom;
    ArmorColor color = ArmorColor::NONE;
    cv::Point2f axis;
    double length = 0;
    double width = 0;
    float tilt_angle = 0;
    size_t id;
    bool laji = true;
};
struct Armor {
    ArmorColor color = ArmorColor::NONE;
    ArmorClass number = ArmorClass::UNKNOWN;
    ArmorKeyPoints2D key_points;

    ISO3 pose;
    bool has_tidy = false;
    struct CvCtx {
        Light left;
        Light right;
        cv::Point2f center;
        double ratio; // 两灯条的中点连线与长灯条的长度之比

        ArmorKeyPoints2D key_points;
        bool duplicated = false;

        CvCtx(const Light& left_light, const Light& right_light):
            left(left_light),
            right(right_light) {
            if (left.corrected) {
                left.top = left.corrected->first;
                left.bottom = left.corrected->second;
            }
            if (right.corrected) {
                right.top = right.corrected->first;
                right.bottom = right.corrected->second;
            }
            center = (left.center + right.center) / 2;
            auto left2right = right.center - left.center;
            auto width = cv::norm(left2right);
            auto max_lightbar_length = std::max(left.length, right.length);
            auto min_lightbar_length = std::min(left.length, right.length);
            ratio = width / max_lightbar_length;
            key_points.points[ArmorKeyPointsIndex::LEFT_TOP] = left.top;
            key_points.points[ArmorKeyPointsIndex::RIGHT_TOP] = right.top;
            key_points.points[ArmorKeyPointsIndex::LEFT_BOTTOM] = left.bottom;
            key_points.points[ArmorKeyPointsIndex::RIGHT_BOTTOM] = right.bottom;
        }
    };
    std::optional<CvCtx> cv;
    struct NetCtx {
        double confidence = 0;
        ArmorColor color = ArmorColor::NONE;
        ArmorClass number = ArmorClass::UNKNOWN;
        ArmorKeyPoints2D key_points;
        std::vector<std::array<std::optional<cv::Point2f>, ArmorKeyPointsIndex::N>> tmp_points;
    };
    std::optional<NetCtx> net;
    struct NumberClassifierCtx {
        ArmorClass number = ArmorClass::UNKNOWN;
        cv::Mat number_img;
        double confidence = 0;
    };
    std::optional<NumberClassifierCtx> number_classifier;
    struct ColorClassifierCtx {
        static constexpr size_t LEFT = 0;
        static constexpr size_t RIGHT = 1;
        std::array<cv::RotatedRect, 2> lights_box;
        std::array<ArmorColor, 2> light_colors = { ArmorColor::NONE, ArmorColor::NONE };
    };
    std::optional<ColorClassifierCtx> color_classifier;
    void tidy() {
        if (net) {
            color = net->color;
            number = net->number;
            key_points = net->key_points;

            if (number_classifier) {
                if (number_classifier->number != ArmorClass::UNKNOWN) {
                    number = number_classifier->number;
                }
            }
            if (color_classifier) {
                auto l = color_classifier->light_colors[ColorClassifierCtx::LEFT];
                auto r = color_classifier->light_colors[ColorClassifierCtx::RIGHT];
                if (l == r) {
                    if (l == ArmorColor::NONE) {
                        if (color != ArmorColor::NONE || color != ArmorColor::PURPLE) {
                            color = ArmorColor::NONE;
                        }
                    } else {
                        color = l;
                    }
                } else if (l == ArmorColor::NONE && r != ArmorColor::NONE) {
                    color = r;
                } else if (r == ArmorColor::NONE && l != ArmorColor::NONE) {
                    color = l;
                }
            }
        } else if (cv) {
            color = cv->left.color;
            number = number_classifier->number;
            key_points = cv->key_points;
        }

        has_tidy = true;
    }
    void add_offset(const cv::Point2f& offset) {
        if (!has_tidy) {
            throw std::runtime_error("addOffset called before tidy");
        }
        key_points.add_offset(offset);
    }
    inline void transform(const Eigen::Matrix<float, 3, 3>& transform_matrix) {
        if (!has_tidy) {
            throw std::runtime_error("transform called before tidy");
        }
        key_points.transform(transform_matrix);
    }
    inline void draw(cv::Mat& img) noexcept {
        if (!has_tidy)
            return;

        auto& pts = key_points.landmarks();

        using I = ArmorKeyPointsIndex;

        auto get = [&](I idx) -> cv::Point {
            auto p = pts[idx];
            // cv::circle(img, p, 5,cv::Scalar(0, 255, 0));
            // cv::putText(img, string_by_armor_key_points_index(std::to_underlying(idx)),p, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
            return p;
        };

        cv::Point lt = get(I::LEFT_TOP);
        cv::Point rt = get(I::RIGHT_TOP);
        cv::Point rb = get(I::RIGHT_BOTTOM);
        cv::Point lb = get(I::LEFT_BOTTOM);
        // cv::Point lm = get(I::LEFT_MID);
        // cv::Point rm = get(I::RIGHT_MID);
        // cv::line(img, lt, rb, cv::Scalar(0, 255, 0), 2);
        // cv::line(img, rb, rt, cv::Scalar(0, 255, 0), 2);
        // cv::line(img, rt, lb, cv::Scalar(0, 255, 0), 2);
        // cv::line(img, lb, lt, cv::Scalar(0, 255, 0), 2);
        // cv::circle(img, lm, 5, cv::Scalar(0, 255, 0), -1);
        // cv::circle(img, rm, 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, lt, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, rt, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, rb, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, lb, 3, cv::Scalar(0, 255, 0), -1);
        cv::Point bottom_center = (lb + rb) * 0.5;

        bottom_center.y += 20;

        std::string text = get_str();

        int font = cv::FONT_HERSHEY_COMPLEX;
        double scale = 0.5;
        int thickness = 1;

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, font, scale, thickness, &baseline);

        cv::Point text_org(
            bottom_center.x - text_size.width / 2,
            bottom_center.y + text_size.height / 2
        );

        cv::putText(img, text, text_org, font, scale, CV_color_by_armor_class(color), thickness);
    }
    inline std::string get_str() const noexcept {
        return string_by_armor_color(color) + "_" + string_by_armor_class(number);
    }
    Armor() = default;
};

struct Armors {
    TimePoint timestamp;
    int id = -1;
    int frame_id = -1;
    std::vector<Armor> armors;
    std::vector<Light> lights;

    inline void draw(cv::Mat& img) noexcept {
        for (auto& armor: armors) {
            armor.draw(img);
        }
        for (auto& light: lights) {
            light.draw(img);
        }
    }
};
} // namespace awakening::auto_aim