#include "rune_detector.hpp"
#include "rune_infer.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/common.hpp"
#include "utils/common/image.hpp"
#include <cstdlib>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <vector>
#if USE_OPENVINO
    #include "utils/net_detector/openvino/net_detector_openvino.hpp"
#endif
#ifdef USE_TRT
    #include "utils/net_detector/tensorrt/net_detector_tensorrt.hpp"
#endif

namespace awakening::auto_buff {
struct RuneDetector::Impl {
    static constexpr const char* OPENVINO = "openvino";
    static constexpr const char* TENSORRT = "tensorrt";
    struct Params {
        struct CvParams {
            int bin_threshold;
            int color_diff_thresh;
            double rune_r_min_area;
            double rune_r_max_area;
            double rune_r_1x1ratio_tol;
            double rune_r_fill_ratio_min;
            double rune_pan_min_area;
            double rune_pan_max_area;
            double rune_pan_cluster_radius;
            double rune_pan_max_square_ratio;
            void load(const YAML::Node& config) {
                bin_threshold = config["bin_threshold"].as<int>();
                color_diff_thresh = config["color_diff_thresh"].as<int>();
                rune_r_min_area = config["rune_r_min_area"].as<double>();
                rune_r_max_area = config["rune_r_max_area"].as<double>();
                rune_r_1x1ratio_tol = config["rune_r_1x1ratio_tol"].as<double>();
                rune_r_fill_ratio_min = config["rune_r_fill_ratio_min"].as<double>();
                rune_pan_min_area = config["rune_pan_min_area"].as<double>();
                rune_pan_max_area = config["rune_pan_max_area"].as<double>();
                rune_pan_cluster_radius = config["rune_pan_cluster_radius"].as<double>();
                rune_pan_max_square_ratio = config["rune_pan_max_square_ratio"].as<double>();
            }
        } cv_params;

        void load(const YAML::Node& config) {
            cv_params.load(config["cv"]);
        }
    } params_;
    struct CVCtx {
        cv::Mat bin;
        cv::Mat src;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<bool> used_flags;
    };
    Impl(const YAML::Node& config) {
        params_.load(config);
        auto backend = config["backend"].as<std::string>();
        if (backend != "opencv") {
            rune_infer_ = RuneInfer::create(config["net_detector"]["rune_infer"]);

            const double scale = rune_infer_->use_norm() ? 1.0 / 255.0f : 1.0f;
            auto format = rune_infer_->target_format();
            auto net_cfg = utils::NetDetectorBase::Config {
                .target_format = format,
                .preprocess_scale = scale,
                .target_w = rune_infer_->input_w(),
                .target_h = rune_infer_->input_h(),
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
    static std::vector<cv::Point> normalizeContour(const std::vector<cv::Point>& cnt) {
        cv::Rect box = cv::boundingRect(cnt);
        std::vector<cv::Point> out;
        out.reserve(cnt.size());

        for (auto& p: cnt) {
            float nx = float(p.x - box.x) / float(box.width);
            float ny = float(p.y - box.y) / float(box.height);
            out.emplace_back(int(nx * 1000), int(ny * 1000));
        }
        return out;
    }
    cv::Mat
    preprocess(const cv::Mat& src, PixelFormat format, EnemyColor enemy_color) const noexcept {
        cv::Mat bin;
        // std::vector<cv::Mat> ch;
        // cv::split(src, ch);
        // cv::Mat b, r;
        // if (format == PixelFormat::RGB) {
        //     b = ch[2];
        //     r = ch[0];
        // } else if (format == PixelFormat::BGR) {
        //     b = ch[0];
        //     r = ch[2];
        // }
        // if (enemy_color == EnemyColor::RED) {
        //     cv::subtract(b, r, bin);
        // } else {
        //     cv::subtract(r, b, bin); // B - R
        // }
        // cv::threshold(bin, bin, 170, 255, cv::THRESH_BINARY);
        if (format == PixelFormat::RGB) {
            cv::cvtColor(src, bin, cv::COLOR_RGB2GRAY);
        } else if (format == PixelFormat::BGR) {
            cv::cvtColor(src, bin, cv::COLOR_BGR2GRAY);
        } else {
            bin = src;
        }
        cv::threshold(bin, bin, params_.cv_params.bin_threshold, 255, cv::THRESH_BINARY);
        int ksize = 3;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        // // cv::erode(bin, bin, kernel, cv::Point(-1, -1), 1);
        // cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);
        // ksize = 3;
        // kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
        // cv::dilate(bin, bin, kernel, cv::Point(-1, -1), 1);

        // cv::namedWindow("Binary Image", cv::WINDOW_NORMAL);
        // cv::resizeWindow("Binary Image", 640, 480);
        // cv::imshow("Binary Image", bin);
        // cv::waitKey(1);
        return bin;
    };
    RuneColor get_color(const cv::Mat& src, const cv::Rect& rect, PixelFormat format) const {
        if (rect.area() <= 0) {
            return RuneColor::NONE;
        }
        if (rect.x < 0 || rect.y < 0 || rect.x >= src.cols || rect.y >= src.rows) {
            return RuneColor::NONE;
        }

        int x2 = rect.x + rect.width;
        int y2 = rect.y + rect.height;

        if (x2 <= 0 || y2 <= 0 || x2 > src.cols || y2 > src.rows) {
            return RuneColor::NONE;
        }
        const cv::Mat roi = src(rect);
        const cv::Scalar avg = cv::mean(roi);
        double B, G, R;
        switch (format) {
            case PixelFormat::BGR:
                B = avg[0];
                G = avg[1];
                R = avg[2];
                break;
            case PixelFormat::RGB:
                R = avg[0];
                G = avg[1];
                B = avg[2];
                break;
            default:
                B = avg[0];
                G = avg[1];
                R = avg[2];
                break;
        }

        if (std::abs(R - B) < params_.cv_params.color_diff_thresh) {
            return RuneColor::NONE;
        }
        if (R > B) {
            return RuneColor::RED;
        } else {
            return RuneColor::BLUE;
        }
    }
    void color_filter(CVCtx& cv, PixelFormat format, EnemyColor enemy_color) const noexcept {
        bool need_red = enemy_color == EnemyColor::BLUE;
        for (int i = 0; i < cv.contours.size(); i++) {
            cv::Rect2f r = cv::boundingRect(cv.contours[i]);
            if (r.width < 5 || r.height < 5)
                continue;

            cv::Rect2f rr = r & cv::Rect2f(0, 0, cv.src.cols, cv.src.rows);
            if (rr.width < 2 || rr.height < 2)
                continue;

            auto color = get_color(cv.src, rr, format);
            bool invalid = false;

            if (need_red) {
                if (color != RuneColor::RED) {
                    cv.used_flags[i] = true;
                }
            } else {
                if (color != RuneColor::BLUE) {
                    cv.used_flags[i] = true;
                }
            }
        }
    }
    std::vector<int> get_child_contours(int idx, const std::vector<cv::Vec4i>& hierarchy) const {
        std::vector<int> children;

        int child = hierarchy[idx][2]; // first child

        while (child != -1) {
            children.push_back(child);
            child = hierarchy[child][0]; // next sibling
        }

        return children;
    }
    std::vector<RuneR> get_rune_rs(CVCtx& cv, cv::Rect focus) const noexcept {
        std::vector<RuneR> result;
        for (int i = 0; i < (int)cv.contours.size(); i++) {
            if (cv.used_flags[i])
                continue;

            if (cv.hierarchy[i][3] != -1)
                continue;

            double area = cv::contourArea(cv.contours[i]);
            if (area < params_.cv_params.rune_r_min_area
                || area > params_.cv_params.rune_r_max_area)
                continue;

            cv::RotatedRect rr = cv::minAreaRect(cv.contours[i]);
            if (!focus.contains(rr.center))
                continue;
            float w = rr.size.width;
            float h = rr.size.height;

            double ratio = (w > h ? w / h : h / w);
            if (ratio - 1.0 > params_.cv_params.rune_r_1x1ratio_tol)
                continue;

            double rect_area = w * h;
            double fill_ratio = area / rect_area;
            if (fill_ratio < params_.cv_params.rune_r_fill_ratio_min)
                continue;
            mark_parent(i, cv.hierarchy, cv.used_flags);
            cv.used_flags[i] = true;
            result.emplace_back(RuneR { .rr = rr });
        }

        return result;
    }
    inline int find_top_parent(int idx, const std::vector<cv::Vec4i>& hierarchy) const noexcept {
        int p = hierarchy[idx][3]; // parent
        while (p != -1 && hierarchy[p][3] != -1) {
            p = hierarchy[p][3]; // 一直追溯到最顶层 parent
        }
        return p; // 若 p == -1 表示 contour 本身就是顶层轮廓
    }
    inline void
    mark_parent(int idx, const std::vector<cv::Vec4i>& hierarchy, std::vector<bool>& used_flags)
        const noexcept {
        int p = hierarchy[idx][3]; // parent
        while (p != -1 && hierarchy[p][3] != -1) {
            p = hierarchy[p][3]; // 一直追溯到最顶层 parent
            used_flags[p] = true;
        }
    }
    std::vector<RuneFlowingLight> get_rune_flowings(CVCtx& cv) const noexcept {
        std::vector<RuneFlowingLight> results;

        return results;
    }
    std::vector<RuneFanTarget> get_rune_fan_targets(CVCtx& cv) const noexcept {
        std::vector<RuneFanTarget> results;
        if (cv.hierarchy.empty())
            return results;

        struct Node {
            int idx;
            cv::Point2f center;
            int parent_top_id;
        };

        std::vector<Node> candidates;

        for (int i = 0; i < cv.contours.size(); i++) {
            if (cv.used_flags[i])
                continue;

            const auto& cnt = cv.contours[i];

            double contour_area = cv::contourArea(cnt);
            if (contour_area < params_.cv_params.rune_pan_min_area
                || contour_area > params_.cv_params.rune_pan_max_area)
                continue;

            cv::Moments m = cv::moments(cnt);
            if (m.m00 == 0)
                continue;

            cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
            int top_parent = find_top_parent(i, cv.hierarchy);

            candidates.push_back({ i, center, top_parent });
        }

        if (candidates.size() < 3)
            return results;

        std::unordered_map<int, std::vector<int>> groups;
        for (int i = 0; i < candidates.size(); i++) {
            groups[candidates[i].parent_top_id].push_back(i);
        }

        for (auto& [parent_top_id, idx_list]: groups) {
            // if (parent_top_id == -1)
            //     continue;
            int M = idx_list.size();
            if (M < 3 || M > 7)
                continue;

            std::vector<int> cluster_id(M, -1);
            int cluster_count = 0;

            for (int i = 0; i < M; i++) {
                if (cluster_id[i] != -1)
                    continue;

                cluster_id[i] = cluster_count;

                std::queue<int> q;
                q.push(i);

                while (!q.empty()) {
                    int u = q.front();
                    q.pop();

                    for (int v = 0; v < M; v++) {
                        if (cluster_id[v] != -1)
                            continue;

                        const auto& cu = candidates[idx_list[u]].center;
                        const auto& cv = candidates[idx_list[v]].center;

                        double dx = cu.x - cv.x;
                        double dy = cu.y - cv.y;
                        double dist = std::sqrt(dx * dx + dy * dy);

                        if (dist <= params_.cv_params.rune_pan_cluster_radius) {
                            cluster_id[v] = cluster_count;
                            q.push(v);
                        }
                    }
                }

                cluster_count++;
            }

            std::vector<int> cluster_size(cluster_count, 0);
            for (int id: cluster_id)
                cluster_size[id]++;

            std::vector<std::vector<cv::Point2f>> cluster_points(cluster_count);

            for (int i = 0; i < M; i++) {
                int cid = cluster_id[i];
                if (cid < 0)
                    continue;

                if (cluster_size[cid] >= 3) {
                    cluster_points[cid].push_back(candidates[idx_list[i]].center);
                }
            }

            for (int cid = 0; cid < cluster_count; cid++) {
                if (cluster_points[cid].size() < 3)
                    continue;

                cv::RotatedRect rr = cv::minAreaRect(cluster_points[cid]);

                double w = rr.size.width;
                double h = rr.size.height;

                if (w < 1 || h < 1)
                    continue;

                double ratio = (w > h ? w / h : h / w);
                if (ratio > params_.cv_params.rune_pan_max_square_ratio)
                    continue;

                std::vector<std::pair<double, cv::Point2f>> dist_list;
                dist_list.reserve(cluster_points[cid].size());

                for (auto& p: cluster_points[cid]) {
                    double dx = p.x - rr.center.x;
                    double dy = p.y - rr.center.y;
                    double dist = dx * dx + dy * dy;
                    dist_list.emplace_back(dist, p);
                }

                std::sort(dist_list.begin(), dist_list.end(), [](const auto& a, const auto& b) {
                    return a.first > b.first;
                });

                if (dist_list.size() < 4)
                    continue;
                // const auto& top_contour = cv.contours[parent_top_id];

                // double area = cv::contourArea(top_contour);
                // double perimeter = cv::arcLength(top_contour, true);

                // if (perimeter < 1e-6)
                //     continue;

                // double roundness = 4.0 * CV_PI * area / (perimeter * perimeter);

                // if (roundness < 0.5)
                //     continue;
                RuneFanTarget fan_target;
                fan_target.center = rr.center;
                fan_target.rr = rr;

                for (int i = 0; i < 4; i++) {
                    fan_target.corners[i] = dist_list[i].second;
                }

                results.push_back(fan_target);

                for (int i = 0; i < M; i++) {
                    int cid2 = cluster_id[i];
                    if (cid2 < 0)
                        continue;

                    if (cluster_size[cid2] >= 3) {
                        int contour_index = candidates[idx_list[i]].idx;
                        cv.used_flags[contour_index] = true;
                        mark_parent(contour_index, cv.hierarchy, cv.used_flags);
                    }
                }
            }
        }

        return results;
    }
    RuneDetection
    detect(const CommonFrame& frame, const cv::Rect& focus, EnemyColor enemy_color) const noexcept {
        RuneDetection result;
        cv::Mat roi = frame.img_frame.src_img(focus);
        if (net_detector_) {
            utils::NetDetectorBase::OutPut net_output;
            net_output = net_detector_->detect(roi, frame.img_frame.format);
            if (net_output.outputs.empty()) {
                return result;
            }
            result.fan_blades = rune_infer_->process(net_output.outputs.front());
            for (auto& fan_blade: result.fan_blades) {
                fan_blade.transform(net_output.transform_matrix);
                fan_blade.add_offset(focus.tl());
            }
        }
        CVCtx cv1;
        cv1.src = roi;
        cv1.bin = preprocess(roi, frame.img_frame.format, enemy_color);
        cv::findContours(
            cv1.bin,
            cv1.contours,
            cv1.hierarchy,
            cv::RETR_TREE,
            cv::CHAIN_APPROX_SIMPLE
        );

        cv1.used_flags.assign(cv1.contours.size(), false);
        color_filter(cv1, frame.img_frame.format, enemy_color);
        result.fan_targets = get_rune_fan_targets(cv1);
        result.rune_flowing_lights = get_rune_flowings(cv1);
        auto roi_center = cv::Point2f(focus.width / 2.0, focus.height / 2.0);
        double side = focus.width / 2.0;
        cv::Rect focus_r =
            cv::Rect(roi_center.x - side / 2.0, roi_center.y - side / 2.0, side, side);
        result.rune_rs = get_rune_rs(cv1, focus_r);
        for (auto& rune_r: result.rune_rs) {
            rune_r.color = get_color(cv1.src, rune_r.rr.boundingRect2f(), frame.img_frame.format);
            rune_r.add_offset(focus.tl());
        }
        for (auto& rune_flowing_light: result.rune_flowing_lights) {
            rune_flowing_light.add_offset(focus.tl());
        }
        for (auto& fan_target: result.fan_targets) {
            fan_target.color =
                get_color(cv1.src, fan_target.rr.boundingRect2f(), frame.img_frame.format);
            fan_target.add_offset(focus.tl());
        }
        return result;
    }
    utils::NetDetectorBase::Ptr net_detector_;
    RuneInfer::Ptr rune_infer_;
};
RuneDetector::RuneDetector(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
RuneDetector::~RuneDetector() noexcept {
    _impl.reset();
}
RuneDetection
RuneDetector::detect(const CommonFrame& frame, const cv::Rect& focus, EnemyColor enemy_color) {
    return _impl->detect(frame, focus, enemy_color);
}
} // namespace awakening::auto_buff
