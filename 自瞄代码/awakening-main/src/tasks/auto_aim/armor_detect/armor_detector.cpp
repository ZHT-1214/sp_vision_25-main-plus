#include "armor_detector.hpp"
#include "armor_infer.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/web.hpp"
#include "utils/logger.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <future>
#include <list>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#if USE_OPENVINO
    #include "utils/net_detector/openvino/net_detector_openvino.hpp"
#endif
#ifdef USE_TRT
    #include "utils/net_detector/tensorrt/net_detector_tensorrt.hpp"
#endif
#include <fstream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <utility>
#include <vector>
namespace awakening::auto_aim {
struct ArmorDetector::Impl {
    static constexpr const char* OPENVINO = "openvino";
    static constexpr const char* TENSORRT = "tensorrt";
    struct Params {
        struct NumberClassifierParams {
            std::string model_path;
            void load(const YAML::Node& config) {
                model_path = replace_root_dir(config["model_path"].as<std::string>());
            }
        };
        std::optional<NumberClassifierParams> number_classifier_params;
        struct ColorClassifierParams {
            double diff_threshold = 20.0;
            void load(const YAML::Node& config) {
                diff_threshold = config["diff_threshold"].as<double>();
            }
        };
        std::optional<ColorClassifierParams> color_classifier_params;
        struct CvParams {
            struct LightParams {
                double bin_threshold;
                double net_ref_threshold_tol;
                double min_wh_ratio;
                double max_wh_ratio;
                double max_angle;

                void load(const YAML::Node& config) {
                    bin_threshold = config["bin_threshold"].as<double>();
                    net_ref_threshold_tol = config["net_ref_threshold_tol"].as<double>();
                    min_wh_ratio = config["min_wh_ratio"].as<double>();
                    max_wh_ratio = config["max_wh_ratio"].as<double>();
                    max_angle = config["max_angle"].as<double>();
                }
            } light_params;
            struct ArmorParams {
                double min_ratio;
                double max_ratio;
                void load(const YAML::Node& config) {
                    min_ratio = config["min_ratio"].as<double>();
                    max_ratio = config["max_ratio"].as<double>();
                }
            } armor_params;
            void load(const YAML::Node& config) {
                light_params.load(config["light"]);
                armor_params.load(config["armor"]);
            }
        } cv_params;

        void load(const YAML::Node& config) {
            cv_params.load(config["cv"]);
            if (config["number_classifier"]["enable"].as<bool>()) {
                number_classifier_params = NumberClassifierParams();
                number_classifier_params->load(config["number_classifier"]);
            }
            if (config["color_classifier"]["enable"].as<bool>()) {
                color_classifier_params = ColorClassifierParams();
                color_classifier_params->load(config["color_classifier"]);
            }
        }
    };
    mutable Params params_;
    Impl(const YAML::Node& config) {
        params_.load(config);
        if (params_.number_classifier_params) {
            init_number_classifier();
        }
        auto backend = config["backend"].as<std::string>();
        if (backend != "opencv") {
            armor_infer_ = ArmorInfer::create(config["net_detector"]["armor_infer"]);
            const double scale = armor_infer_->use_norm() ? 1.0 / 255.0f : 1.0f;
            auto format = armor_infer_->target_format();
            auto net_cfg = utils::NetDetectorBase::Config {
                .target_format = format,
                .preprocess_scale = scale,
                .target_w = armor_infer_->input_w(),
                .target_h = armor_infer_->input_h(),
            };
            bool backend_valid = false;
#ifdef USE_OPENVINO
            if (backend == OPENVINO) {
                backend_valid = true;
                net_detector_ = std::make_unique<utils::NetDetectorOpenVINO>(
                    config["net_detector"][OPENVINO],
                    net_cfg
                );
            }
#endif
#ifdef USE_TRT
            if (backend == TENSORRT) {
                backend_valid = true;
                net_detector_ = std::make_unique<utils::NetDetectorTensorrt>(
                    config["net_detector"][TENSORRT],
                    net_cfg
                );
            }
#endif
            if (!backend_valid) {
                throw std::runtime_error("Invalid backend");
            }
        }
    }
    bool extract_number(const cv::Mat& src, Armor& armor) const noexcept {
        // Light length in image
        constexpr int light_length = 12;
        // Image size after warp
        constexpr int warp_height = 28;
        constexpr int small_armor_width = 32;
        constexpr int large_armor_width = 54;
        // Number ROI size
        const cv::Size roi_size(20, 28);
        constexpr float min_large_center_distance = 3.5f;

        if (src.empty() || src.cols < 10 || src.rows < 10) {
            AWAKENING_ERROR("[extractNumber] input src is empty or too small!");
            return false;
        }

        auto key_points = (armor.net.has_value()) ? armor.net->key_points.points
            : (armor.cv.has_value())
            ? armor.cv->key_points.points
            : std::array<std::optional<cv::Point2f>, ArmorKeyPointsIndex::N>();

        const cv::Point2f& rb = key_points[ArmorKeyPointsIndex::RIGHT_BOTTOM].value();
        const cv::Point2f& rt = key_points[ArmorKeyPointsIndex::RIGHT_TOP].value();
        const cv::Point2f& lt = key_points[ArmorKeyPointsIndex::LEFT_TOP].value();
        const cv::Point2f& lb = key_points[ArmorKeyPointsIndex::LEFT_BOTTOM].value();
        const float l1_len = cv::norm(rt - rb);
        const float l2_len = cv::norm(lt - lb);
        const cv::Point2f c1 = (rb + rt) * 0.5f;
        const cv::Point2f c2 = (lb + lt) * 0.5f;

        const float avg_light_len = 0.5f * (l1_len + l2_len);
        const float center_dist = avg_light_len > 1e-3f ? cv::norm(c1 - c2) / avg_light_len : 0.f;

        const bool is_large = center_dist > min_large_center_distance;

        cv::Point2f lights_vertices[4] = { lb, lt, rt, rb };

        const int top_light_y = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width = !is_large ? small_armor_width : large_armor_width;
        cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(0, top_light_y),
            cv::Point(warp_width - 1, top_light_y),
            cv::Point(warp_width - 1, bottom_light_y),
        };
        cv::Mat number_image;
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

        // Get ROI
        number_image =
            number_image(cv::Rect2f(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        armor.number_classifier = Armor::NumberClassifierCtx();
        armor.number_classifier->number_img = std::move(number_image);
        return true;
    }
    void classify_color(const cv::Mat& src, Armor& armor, PixelFormat pixel_format) const noexcept {
        constexpr float light_width = 1.0f;
        constexpr float light_height = 5.0f;
        if (src.empty() || src.cols < 10 || src.rows < 10 || !params_.color_classifier_params) {
            AWAKENING_ERROR("[classifyColor] input src is empty or too small!");
            return;
        }
        auto& key_points = armor.net->key_points.points;
        auto getPt = [&](ArmorKeyPointsIndex idx) -> const cv::Point2f* {
            auto& opt = key_points[idx];
            return opt ? &(*opt) : nullptr;
        };

        const cv::Point2f* rb = getPt(ArmorKeyPointsIndex::RIGHT_BOTTOM);
        const cv::Point2f* rt = getPt(ArmorKeyPointsIndex::RIGHT_TOP);
        const cv::Point2f* lt = getPt(ArmorKeyPointsIndex::LEFT_TOP);
        const cv::Point2f* lb = getPt(ArmorKeyPointsIndex::LEFT_BOTTOM);

        if (!rb || !rt || !lt || !lb)
            return;

        std::array<cv::RotatedRect, 2> lights_box;

        auto makeLight = [&](const cv::Point2f& top, const cv::Point2f& bottom) {
            cv::Point2f center = (top + bottom) * 0.5f;

            float height = cv::norm(top - bottom);
            float width = height / (light_height / light_width);

            float angle = std::atan2(top.y - bottom.y, top.x - bottom.x) * 180.0f / CV_PI;
            if (width > height) {
                std::swap(width, height);
                angle += 90.0f;
            }

            return cv::RotatedRect(center, cv::Size2f(width, height), angle);
        };

        lights_box[Armor::ColorClassifierCtx::RIGHT] = makeLight(*rt, *rb);
        lights_box[Armor::ColorClassifierCtx::LEFT] = makeLight(*lt, *lb);

        armor.color_classifier = Armor::ColorClassifierCtx();
        armor.color_classifier->lights_box = lights_box;

        armor.color_classifier = Armor::ColorClassifierCtx();
        armor.color_classifier->lights_box = lights_box;
        auto extractRotatedROI = [](const cv::Mat& src, const cv::RotatedRect& rect) {
            cv::Rect2f bbox = rect.boundingRect();
            bbox &= cv::Rect2f(0, 0, src.cols, src.rows);
            if (bbox.width <= 0 || bbox.height <= 0)
                return cv::Mat();
            return src(bbox);
        };

        auto judgeColor = [&](const cv::Mat& roi) {
            if (roi.empty() || roi.channels() < 3)
                return ArmorColor::NONE;
            cv::Scalar mean_val = cv::mean(roi);
            float R = 0.f, B = 0.f;
            switch (pixel_format) {
                case PixelFormat::BGR:
                    R = mean_val[2];
                    B = mean_val[0];
                    break;
                case PixelFormat::RGB:
                    B = mean_val[2];
                    R = mean_val[0];
                    break;
                case PixelFormat::GRAY:
                    return ArmorColor::NONE;
            }

            const float threshold = params_.color_classifier_params->diff_threshold;
            if (R - B > threshold)
                return ArmorColor::RED;
            if (B - R > threshold)
                return ArmorColor::BLUE;
            return ArmorColor::NONE;
        };
        for (int i = 0; i < 2; ++i) {
            cv::Mat roi = extractRotatedROI(src, lights_box[i]);
            armor.color_classifier->light_colors[i] = judgeColor(roi);
        }

        return;
    }

    void init_number_classifier() {
        if (!params_.number_classifier_params) {
            return;
        }
        const std::string model_path = params_.number_classifier_params->model_path;
        std::unique_ptr<cv::dnn::Net> number_net_ =
            std::make_unique<cv::dnn::Net>(cv::dnn::readNetFromONNX(model_path));

        if (number_net_->empty()) {
            throw std::runtime_error("Failed to load number classifier model" + model_path);
        } else {
            AWAKENING_DEBUG("Successfully loaded number classifier model from {}", model_path);
        }
        number_net_.reset();
    }
    bool classify_number_batch(std::vector<Armor*>& armors) const noexcept {
        static thread_local std::unique_ptr<cv::dnn::Net> thread_net;

        if (!thread_net) {
            thread_net = std::make_unique<cv::dnn::Net>(
                cv::dnn::readNetFromONNX(params_.number_classifier_params->model_path)
            );
            AWAKENING_DEBUG("Created thread-local number classifier model");
            if (thread_net->empty()) {
                AWAKENING_ERROR("Failed to load model");
                return false;
            }
        }

        std::vector<cv::Mat> images;
        std::vector<Armor*> valid_armors;

        for (auto* armor: armors) {
            if (armor && armor->number_classifier && !armor->number_classifier->number_img.empty())
            {
                images.emplace_back(armor->number_classifier->number_img);
                valid_armors.emplace_back(armor);
            }
        }

        if (images.empty())
            return false;

        cv::Mat blob;
        cv::dnn::blobFromImages(images, blob, 1.0 / 255.0);

        thread_net->setInput(blob);
        cv::Mat outputs = thread_net->forward();

        static const std::array<ArmorClass, 8> label_map = {
            ArmorClass::NO1, ArmorClass::NO2,     ArmorClass::NO3,    ArmorClass::NO4,
            ArmorClass::NO5, ArmorClass::OUTPOST, ArmorClass::SENTRY, ArmorClass::BASE
        };

        for (int i = 0; i < outputs.rows; ++i) {
            cv::Mat logits = outputs.row(i);

            double max_val;
            cv::minMaxLoc(logits, nullptr, &max_val);

            cv::Mat prob;
            cv::exp(logits - max_val, prob);
            prob /= cv::sum(prob)[0];

            double confidence;
            cv::Point class_id;
            cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &class_id);

            int label = class_id.x;

            auto* armor = valid_armors[i];
            armor->number_classifier->confidence = confidence;

            if (label >= 0 && label < (int)label_map.size()) {
                armor->number_classifier->number = label_map[label];
            } else {
                armor->number_classifier->number = ArmorClass::UNKNOWN;
            }
        }

        return true;
    }
    bool is_light(const Light& light) const noexcept {
        const float ratio = light.width / light.length;

        if (ratio <= params_.cv_params.light_params.min_wh_ratio
            || ratio >= params_.cv_params.light_params.max_wh_ratio)
            return false;

        if (light.tilt_angle >= params_.cv_params.light_params.max_angle)
            return false;

        return true;
    }
    void
    correct_corners(Light& light, const cv::Mat& gray) const noexcept { //copy form sp_vision_25
        // 参数保护
        if (gray.empty() || light.length < 2.f || light.width < 1.f)
            return;

        constexpr float MAX_BRIGHTNESS = 25.f; // 归一化最大亮度值
        constexpr float ROI_SCALE = 0.07f; // ROI扩展比例
        constexpr float SEARCH_START = 0.4f; // 搜索起始位置比例
        constexpr float SEARCH_END = 0.6f; // 搜索结束位置比例

        // 扩展ROI
        cv::Rect2f roi_box = light.boundingRect();
        roi_box.x -= static_cast<int>(roi_box.width * ROI_SCALE);
        roi_box.y -= static_cast<int>(roi_box.height * ROI_SCALE);
        roi_box.width += static_cast<int>(2 * roi_box.width * ROI_SCALE);
        roi_box.height += static_cast<int>(2 * roi_box.height * ROI_SCALE);

        // 边界约束
        roi_box &= cv::Rect2f(0, 0, gray.cols, gray.rows);
        if (roi_box.width <= 0 || roi_box.height <= 0)
            return;

        // 提取ROI并归一化
        cv::Mat roi = gray(roi_box);
        const float mean_val = static_cast<float>(cv::mean(roi)[0]);
        roi.convertTo(roi, CV_32F);
        cv::normalize(roi, roi, 0.f, MAX_BRIGHTNESS, cv::NORM_MINMAX);

        // 计算质心
        const cv::Moments moments = cv::moments(roi);
        if (std::abs(moments.m00) < 1e-6f)
            return; // 避免除零

        const cv::Point2f centroid(
            moments.m10 / moments.m00 + roi_box.x,
            moments.m01 / moments.m00 + roi_box.y
        );

        // 生成稀疏点云（优化性能）
        std::vector<cv::Point2f> points;
        for (int i = 0; i < roi.rows; ++i) {
            for (int j = 0; j < roi.cols; ++j) {
                const float weight = roi.at<float>(i, j);
                if (weight > 1e-3f) {
                    points.emplace_back(static_cast<float>(j), static_cast<float>(i));
                }
            }
        }
        if (points.size() < 2)
            return; // PCA需要至少两个点

        // PCA计算对称轴方向
        cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
        cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
        float axis_norm = cv::norm(axis);
        if (axis_norm < 1e-6f)
            return; // 避免零向量
        axis /= axis_norm;
        if (axis.y > 0)
            axis = -axis; // 统一方向

        // 搜索角点
        const auto find_corner = [&](int direction, cv::Point2f raw) -> cv::Point2f {
            const float dx = axis.x * direction;
            const float dy = axis.y * direction;
            const float search_length = light.length * (SEARCH_END - SEARCH_START);

            std::vector<cv::Point2f> candidates;
            const int half_width = std::max(0, static_cast<int>((light.width - 2.f) * 0.5f));

            for (int i_offset = -half_width; i_offset <= half_width; ++i_offset) {
                cv::Point2f start_point(
                    centroid.x + light.length * SEARCH_START * dx + i_offset,
                    centroid.y + light.length * SEARCH_START * dy
                );

                cv::Point2f corner = start_point;
                float max_diff = 0.f;
                bool found = false;

                for (float step = 0.f; step < search_length; step += 1.f) {
                    cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);
                    cv::Point2i cur_pt(
                        static_cast<int>(cur_point.x),
                        static_cast<int>(cur_point.y)
                    );
                    cv::Point2i prev_pt(
                        static_cast<int>(cur_point.x - dx),
                        static_cast<int>(cur_point.y - dy)
                    );
                    if (cur_pt.x < 0 || cur_pt.x >= gray.cols || cur_pt.y < 0
                        || cur_pt.y >= gray.rows)
                        break;
                    if (prev_pt.x < 0 || prev_pt.x >= gray.cols || prev_pt.y < 0
                        || prev_pt.y >= gray.rows)
                        continue;
                    const float prev_val = static_cast<float>(gray.at<uchar>(prev_pt));
                    const float cur_val = static_cast<float>(gray.at<uchar>(cur_pt));
                    const float diff = prev_val - cur_val;
                    if (diff > max_diff && prev_val > mean_val) {
                        max_diff = diff;
                        corner = cv::Point2f(prev_pt.x, prev_pt.y); // 跳变点
                        found = true;
                    }
                }
                if (found) {
                    candidates.push_back(corner);
                }
            }

            if (candidates.empty())
                return raw;

            cv::Point2f sum_pt(0.f, 0.f);
            for (const auto& pt: candidates)
                sum_pt += pt;

            return sum_pt / static_cast<float>(candidates.size());
        };
        light.corrected.emplace();
        light.corrected->first = find_corner(1, light.top);
        light.corrected->second = find_corner(-1, light.bottom);
    }
    std::vector<Light> detect_lights(
        const cv::Mat& src,
        PixelFormat format,
        cv::Rect bbox,
        const std::optional<cv::Mat>& _gray = std::nullopt
    ) const noexcept {
        const auto detect_roi = src(bbox);
        cv::Mat gray;
        if (_gray.has_value()) {
            gray = _gray.value();
        } else {
            cv::cvtColor(detect_roi, gray, cv::COLOR_BGR2GRAY);
        }
        cv::Mat gray_img, bin;
        cv::cvtColor(detect_roi, gray_img, cv::COLOR_BGR2GRAY);
        cv::threshold(
            gray,
            bin,
            params_.cv_params.light_params.bin_threshold,
            255,
            cv::THRESH_BINARY
        );
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<Light> lights;
        lights.reserve(contours.size());
        const bool use_bgr = (format == PixelFormat::BGR);
        const int r_idx = use_bgr ? 2 : 0;
        const int b_idx = use_bgr ? 0 : 2;
        size_t l_id = 0;
        for (const auto& contour: contours) {
            const int n = static_cast<int>(contour.size());
            if (n < 6)
                continue;

            Light light(contour, l_id++);
            if (!is_light(light))
                continue;

            int sum_r = 0, sum_b = 0;
            for (const auto& pt: contour) {
                const uchar* pix = detect_roi.ptr<uchar>(pt.y) + pt.x * 3;
                sum_r += pix[r_idx];
                sum_b += pix[b_idx];
            }

            const int avg_diff = std::abs(sum_r - sum_b) / n;
            if (avg_diff <= params_.color_classifier_params->diff_threshold)
                continue;

            light.color = (sum_r > sum_b) ? ArmorColor::RED : ArmorColor::BLUE;
            if (light.color != ArmorColor::NONE) {
                correct_corners(light, gray);
                lights.emplace_back(std::move(light));
            }
        }
        // static Web web;
        // // cv::cvtColor(bin, bin, cv::COLOR_GRAY2BGR);
        // web.write_shm(bin);
        return lights;
    }
    int cal_light_threshold(
        const cv::Mat& gray,
        const std::vector<std::pair<cv::Point2f, cv::Point2f>>& lights
    ) const {
        if (lights.empty())
            return 0;
        std::vector<int> vals(lights.size());
        cv::parallel_for_(cv::Range(0, (int)lights.size()), [&](const cv::Range& range) {
            for (int idx = range.start; idx < range.end; ++idx) {
                const auto& light = lights[idx];
                const cv::Point2f& p1 = light.first;
                const cv::Point2f& p2 = light.second;
                cv::Point2f d = p2 - p1;
                float dx = d.x, dy = d.y;
                float len2 = dx * dx + dy * dy;
                int samples = std::max(2, (int)std::sqrt(len2));
                double sum = 0.0;
                int count = 0;
                for (int j = 0; j < samples; ++j) {
                    float t = (float)j / (samples - 1);
                    float px = p1.x + t * dx;
                    float py = p1.y + t * dy;
                    int x = (int)std::round(px);
                    int y = (int)std::round(py);
                    if ((unsigned)x >= (unsigned)gray.cols || (unsigned)y >= (unsigned)gray.rows)
                        continue;
                    const uchar* row = gray.ptr<uchar>(y);
                    sum += row[x];
                    ++count;
                }
                vals[idx] = (count > 0) ? (int)(sum / count) : 0;
            }
        });
        double total = 0;
        for (int v: vals)
            total += v;
        return (int)(total / vals.size());
    }
    std::tuple<std::vector<Light>, std::vector<Armor>> detect_net(
        const CommonFrame& frame,
        const cv::Rect& net_focus,
        const std::optional<cv::Rect>& detect_light
    ) const {
        const auto& src_img = frame.img_frame.src_img;
        const auto roi = src_img(net_focus);
        utils::NetDetectorBase::OutPut net_output;
        std::vector<Light> lights;
        std::vector<Armor> result;

        net_output = net_detector_->detect(roi, frame.img_frame.format);
        if (net_output.outputs.empty()) {
            return { lights, result };
        }
        result = armor_infer_->process(net_output.outputs.front());

        if (!net_output.resized_img.empty()) {
            if (params_.number_classifier_params) {
                std::vector<Armor*> batch_armors;
                batch_armors.reserve(result.size());
                for (auto& armor: result) {
                    if (extract_number(net_output.resized_img, armor)) {
                        batch_armors.push_back(&armor);
                    }
                }
                classify_number_batch(batch_armors);
            }

            for (auto& armor: result) {
                if (params_.color_classifier_params) {
                    classify_color(net_output.resized_img, armor, frame.img_frame.format);
                }
                armor.tidy();
                armor.transform(net_output.transform_matrix);
                armor.add_offset(net_focus.tl());
            }
        }

        if (detect_light) {
            cv::Mat gray;
            cv::cvtColor(frame.img_frame.src_img(*detect_light), gray, cv::COLOR_BGR2GRAY);
            if (params_.cv_params.light_params.net_ref_threshold_tol > 0) {
                std::vector<std::pair<cv::Point2f, cv::Point2f>> net_lights;
                auto trans_local = [&](const cv::Point2f& pt) {
                    return (pt - cv::Point2f(detect_light->tl()));
                };
                for (auto& armor: result) {
                    if (armor.color_classifier) {
                        auto l =
                            armor.color_classifier->light_colors[Armor::ColorClassifierCtx::LEFT];
                        auto r =
                            armor.color_classifier->light_colors[Armor::ColorClassifierCtx::RIGHT];
                        auto key_points = armor.key_points.landmarks();
                        if (l != ArmorColor::NONE) {
                            net_lights.emplace_back(
                                trans_local(key_points[ArmorKeyPointsIndex::LEFT_TOP]),
                                trans_local(key_points[ArmorKeyPointsIndex::LEFT_BOTTOM])
                            );
                        }
                        if (r != ArmorColor::NONE) {
                            net_lights.emplace_back(
                                trans_local(key_points[ArmorKeyPointsIndex::RIGHT_TOP]),
                                trans_local(key_points[ArmorKeyPointsIndex::RIGHT_BOTTOM])
                            );
                        }
                    }
                }
                auto threshold = cal_light_threshold(gray, net_lights);
                params_.cv_params.light_params.bin_threshold =
                    threshold - params_.cv_params.light_params.net_ref_threshold_tol;
            }

            lights = detect_lights(
                frame.img_frame.src_img,
                frame.img_frame.format,
                *detect_light,
                std::make_optional<cv::Mat>(gray)
            );
            for (auto& light: lights) {
                light.add_offset(detect_light->tl());
            }
        }

        return { std::move(lights), std::move(result) };
    }
    bool is_armor(const Armor& armor
    ) const { //非常简单少量的参数，直接完全信下游的数字分类，反正batch一下才1ms
        auto ratio_ok = armor.cv->ratio > params_.cv_params.armor_params.min_ratio
            && armor.cv->ratio < params_.cv_params.armor_params.max_ratio;
        return ratio_ok;
    }

    std::tuple<std::vector<Light>, std::vector<Armor>>
    detect_cv(const CommonFrame& frame, const std::optional<cv::Rect>& detect_light) const {
        const auto& src_img = frame.img_frame.src_img;
        auto bbox = detect_light
            ? detect_light.value()
            : cv::Rect(0, 0, frame.img_frame.src_img.cols, frame.img_frame.src_img.rows);

        std::vector<Light> lights;
        std::vector<Armor> result;

        lights = detect_lights(frame.img_frame.src_img, frame.img_frame.format, bbox);
        for (auto& light: lights) {
            light.add_offset(bbox.tl());
        }
        std::sort(lights.begin(), lights.end(), [](const Light& a, const Light& b) {
            return a.center.x < b.center.x;
        });
        std::list<Armor> tmp_armors;
        for (auto left = lights.begin(); left != lights.end(); left++) { //copy form sp_vision_25
            for (auto right = std::next(left); right != lights.end(); right++) {
                if (left->color != right->color) {
                    continue;
                }
                Armor armor;
                armor.cv.emplace(*left, *right);
                if (!is_armor(armor)) {
                    continue;
                }
                if (!extract_number(src_img, armor)) {
                    continue;
                }
                tmp_armors.emplace_back(std::move(armor));
            }
        }
        std::vector<Armor*> batch_armors;
        batch_armors.reserve(tmp_armors.size());
        for (auto& armor: tmp_armors) {
            batch_armors.push_back(&armor);
        }
        classify_number_batch(batch_armors);
        tmp_armors.remove_if([](const Armor& a) {
            return a.number_classifier->number == ArmorClass::UNKNOWN;
        });
        for (auto& armor: tmp_armors) {
            if (!armor.cv->duplicated) {
                armor.tidy();
                result.push_back(armor);
            }
        }
        return { std::move(lights), std::move(result) };
    }
    std::tuple<std::vector<Light>, std::vector<Armor>> detect(
        const CommonFrame& frame,
        const cv::Rect& net_focus,
        const std::optional<cv::Rect>& detect_light = std::nullopt
    ) const {
        if (net_detector_) {
            return detect_net(frame, net_focus, detect_light);
        } else {
            return detect_cv(frame, detect_light);
        }
    }
    double get_net_wh_ratio() const noexcept {
        if (net_detector_) {
            return armor_infer_->input_w() / static_cast<double>(armor_infer_->input_h());
        }
        return 1.0;
    }

    utils::NetDetectorBase::Ptr net_detector_;
    ArmorInfer::Ptr armor_infer_;
};
ArmorDetector::ArmorDetector(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
ArmorDetector::~ArmorDetector() noexcept {
    _impl.reset();
}
std::tuple<std::vector<Light>, std::vector<Armor>> ArmorDetector::detect(
    const CommonFrame& frame,
    const cv::Rect& net_focus,
    const std::optional<cv::Rect>& detect_light
) {
    return _impl->detect(frame, net_focus, detect_light);
}
double ArmorDetector::get_net_wh_ratio() const noexcept {
    return _impl->get_net_wh_ratio();
}
} // namespace awakening::auto_aim
