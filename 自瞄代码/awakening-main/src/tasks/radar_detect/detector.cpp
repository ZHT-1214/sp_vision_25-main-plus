#include "detector.hpp"
#include "utils/net_detector/tensorrt/net_detector_tensorrt.hpp"
#include <array>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <yaml-cpp/node/node.h>

namespace awakening::radar_detect {
struct Detector::Impl {
    static constexpr int num_armor_class = 7;
    struct Params {
        double car_confidence_threshold;
        double armor_confidence_threshold;
        double armor_color_diff_threshold;
        void load(const YAML::Node& config) {
            car_confidence_threshold = config["car_confidence_threshold"].as<double>();
            armor_confidence_threshold = config["armor_confidence_threshold"].as<double>();
            armor_color_diff_threshold = config["armor_color_diff_threshold"].as<double>();
        }
    } params_;
    Impl(const YAML::Node& config) {
        params_.load(config);
        auto net_cfg = utils::NetDetectorBase::Config {
            .target_format = PixelFormat::RGB,
            .preprocess_scale = 1.0 / 255.0,
            .target_w = 1280,
            .target_h = 1280,
        };
        car_trt_ = std::make_unique<utils::NetDetectorTensorrt>(config["car"], net_cfg);
        armor_trt_ = std::make_unique<utils::NetDetectorTensorrt>(config["armor"], net_cfg);
    }
    std::vector<Car> post_process_car(const cv::Mat& output) {
        std::vector<Car> cars;
        if (output.empty()) {
            return cars;
        }
        for (int i = 0; i < output.cols; ++i) {
            float conf = output.ptr<float>(4)[i];
            if (conf < params_.car_confidence_threshold) {
                continue;
            }
            float x = output.ptr<float>(0)[i]; // 第0行第i列
            float y = output.ptr<float>(1)[i];
            float w = output.ptr<float>(2)[i];
            float h = output.ptr<float>(3)[i];
            Car car;
            car.confidence = conf;
            car.bbox = cv::Rect2f(x - w / 2, y - h / 2, w, h);
            cars.push_back(car);
        }
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;

        for (const auto& car: cars) {
            boxes.push_back(car.bbox);
            confidences.push_back(car.confidence);
        }

        std::vector<int> indices;

        cv::dnn::NMSBoxes(boxes, confidences, 0.2, 0.35, indices);
        std::vector<Car> result;
        for (int idx: indices) {
            result.push_back(cars[idx]);
        }
        return result;
    }
    std::vector<Armor> armor_post_process(const cv::Mat& output) {
        std::vector<Armor> armors;
        if (output.empty()) {
            return armors;
        }
        for (int i = 0; i < output.cols; ++i) {
            double max_score = -1.0;
            int max_class = 0;
            std::array<double, num_armor_class> scores;
            for (int j = 0; j < num_armor_class; ++j) {
                scores[j] = output.ptr<float>(4 + j)[i];
                if (scores[j] > max_score) {
                    max_score = scores[j];
                    max_class = j;
                }
            }
            if (max_score < params_.armor_confidence_threshold) {
                continue;
            }
            float x = output.ptr<float>(0)[i]; // 第0行第i列
            float y = output.ptr<float>(1)[i];
            float w = output.ptr<float>(2)[i];
            float h = output.ptr<float>(3)[i];
            Armor armor;
            armor.confidence = max_score;
            armor.bbox = cv::Rect2f(x - w / 2, y - h / 2, w, h);
            armor.number = ArmorClass(max_class);
            armors.push_back(armor);
        }
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;

        for (const auto& armor: armors) {
            boxes.push_back(armor.bbox);
            confidences.push_back(armor.confidence);
        }

        std::vector<int> indices;

        cv::dnn::NMSBoxes(boxes, confidences, params_.armor_confidence_threshold, 0.35, indices);
        std::vector<Armor> result;
        for (int idx: indices) {
            result.push_back(armors[idx]);
        }
        return result;
    }
    std::vector<Armor> detect_armors(const CommonFrame& frame, const cv::Rect& focus) {
        std::vector<Armor> armors;
        if (frame.img_frame.src_img.empty()) {
            return armors;
        }
        const auto& src_img = frame.img_frame.src_img;

        cv::Rect src_rect(0, 0, src_img.cols, src_img.rows);
        cv::Rect safe_expanded = focus & src_rect;
        if (safe_expanded.area() <= 0) {
            return armors;
        }

        const auto roi = src_img(safe_expanded);
        cv::Rect2f roi_safe_bounds(0, 0, roi.cols, roi.rows);
        auto armor_output = armor_trt_->detect(roi, frame.img_frame.format);
        if (armor_output.outputs.empty()) {
            return armors;
        }
        armors = armor_post_process(armor_output.outputs.front());
        for (auto& armor: armors) {
            armor.bbox = utils::transform_rect(armor_output.transform_matrix, armor.bbox);
            armor.color = get_color(
                roi(cv::Rect2f(armor.bbox) & roi_safe_bounds),
                params_.armor_color_diff_threshold,
                frame.img_frame.format
            );
            armor.bbox += cv::Point2f(focus.x, focus.y);
        }
        // Implementation for detecting armors
        return armors;
    }
    std::vector<Car> detect(const CommonFrame& frame, const cv::Rect& focus) {
        std::vector<Car> cars;
        if (frame.img_frame.src_img.empty()) {
            return cars;
        }

        const auto& src_img = frame.img_frame.src_img;

        cv::Rect src_rect(0, 0, src_img.cols, src_img.rows);
        cv::Rect safe_expanded = focus & src_rect;
        if (safe_expanded.area() <= 0) {
            return cars;
        }

        const auto roi = src_img(safe_expanded);
        auto car_output = car_trt_->detect(roi, frame.img_frame.format);
        if (car_output.outputs.empty()) {
            return cars;
        }
        cars = post_process_car(car_output.outputs.front());

        struct Ctx {
            Car* car;
            cv::Mat img;
            cv::Rect2f raw_bbox;
            double scale_x;
            double scale_y;
            cv::Rect2f bbox_in_concatenated;
        };
        std::vector<Ctx> car_ctxs;
        cv::Rect2f roi_safe_bounds(0, 0, roi.cols, roi.rows);

        for (auto& car: cars) {
            car.bbox = utils::transform_rect(car_output.transform_matrix, car.bbox);
            car.timestamp = frame.img_frame.timestamp;
            cv::Rect2f clamped_bbox = cv::Rect2f(car.bbox) & roi_safe_bounds;

            if (clamped_bbox.area() <= 0) {
                continue;
            }

            car_ctxs.emplace_back(
                Ctx { .car = &car, .img = roi(clamped_bbox), .raw_bbox = car.bbox }
            );
        }

        if (car_ctxs.empty()) {
            return cars;
        }

        int img_width = car_ctxs[0].img.cols;
        int img_height = car_ctxs[0].img.rows;

        if (img_width <= 0 || img_height <= 0) {
            return cars;
        }

        for (auto& ctx: car_ctxs) {
            ctx.scale_x = ctx.img.size().width / static_cast<double>(img_width);
            ctx.scale_y = ctx.img.size().height / static_cast<double>(img_height);
            if (ctx.img.size() != cv::Size(img_width, img_height)) {
                cv::resize(ctx.img, ctx.img, cv::Size(img_width, img_height));
            }
        }

        int num_imgs = car_ctxs.size();
        int rows = std::ceil(std::sqrt(num_imgs));
        int cols = rows;

        cv::Mat concatenated_img(
            rows * img_height,
            cols * img_width,
            car_ctxs[0].img.type(),
            cv::Scalar(114, 114, 114)
        );

        int idx = 0;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (idx >= num_imgs)
                    break;
                cv::Rect2f roi_rect(c * img_width, r * img_height, img_width, img_height);
                car_ctxs[idx].img.copyTo(concatenated_img(roi_rect));
                car_ctxs[idx].bbox_in_concatenated = roi_rect;
                ++idx;
            }
        }

        auto armor_output = armor_trt_->detect(concatenated_img, frame.img_frame.format);
        if (armor_output.outputs.empty()) {
            return cars;
        }
        auto armors = armor_post_process(armor_output.outputs.front());

        for (auto& armor: armors) {
            armor.bbox = utils::transform_rect(armor_output.transform_matrix, armor.bbox);

            for (int i = 0; i < car_ctxs.size(); ++i) {
                const auto& ctx = car_ctxs[i];
                if (ctx.bbox_in_concatenated.contains((armor.bbox.tl() + armor.bbox.br()) * 0.5)) {
                    armor.bbox.x -= ctx.bbox_in_concatenated.tl().x;
                    armor.bbox.y -= ctx.bbox_in_concatenated.tl().y;
                    armor.bbox.x = armor.bbox.x * ctx.scale_x;
                    armor.bbox.y = armor.bbox.y * ctx.scale_y;
                    armor.bbox.width = armor.bbox.width * ctx.scale_x;
                    armor.bbox.height = armor.bbox.height * ctx.scale_y;
                    armor.bbox.x += ctx.raw_bbox.x;
                    armor.bbox.y += ctx.raw_bbox.y;
                    armor.color = get_color(
                        roi(cv::Rect2f(armor.bbox) & roi_safe_bounds),
                        params_.armor_color_diff_threshold,
                        frame.img_frame.format
                    );

                    ctx.car->armors.push_back(armor);
                    break;
                }
            }
        }

        for (auto& car: cars) {
            car.bbox += cv::Point2f(focus.x, focus.y);
            for (auto& armor: car.armors) {
                armor.bbox += cv::Point2f(focus.x, focus.y);
            }
            car.tidy();
        }
        return cars;
    }

    utils::NetDetectorTensorrt::Ptr car_trt_;
    utils::NetDetectorTensorrt::Ptr armor_trt_;
};
Detector::Detector(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
Detector::~Detector() noexcept {
    _impl.reset();
}
std::vector<Car> Detector::detect(const CommonFrame& frame, const cv::Rect& focus) {
    return _impl->detect(frame, focus);
}
std::vector<Armor> Detector::detect_armors(const CommonFrame& frame, const cv::Rect& focus) {
    return _impl->detect_armors(frame, focus);
}
} // namespace awakening::radar_detect
