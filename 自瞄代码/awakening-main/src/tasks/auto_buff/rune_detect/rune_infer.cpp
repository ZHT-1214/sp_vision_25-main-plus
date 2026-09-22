#include "rune_infer.hpp"
#include "tasks/auto_buff/type.hpp"
#include "utils/utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

namespace awakening::auto_buff {
static constexpr float MERGE_CONF_ERROR = 0.95f;
static constexpr float MERGE_MIN_IOU = 0.9f;
static constexpr float NMS_THRESHOLD = 0.35;
static constexpr int TOP_K = 128;
enum class Mode : int { CB };
inline Mode modeFromString(const std::string& s) noexcept {
    std::string str = utils::to_upper(s);
    if (str == "CB")
        return Mode::CB;
    return Mode::CB;
}
template<Mode M>
struct ModelTraits; // declare
// TUP
template<>
struct ModelTraits<Mode::CB> {
    static constexpr int INPUT_W = 640;
    static constexpr int INPUT_H = 640;
    static constexpr bool USE_NORM = true;
    static constexpr int NUM_POINTS = 9; // raw model output keypoints
    static constexpr int REMAPPED_KPTS = 6; // remapped to old 6-pt layout for solver compatibility
    static constexpr int NUM_CLASSES = 2;
    static constexpr int KPT_DIM = 3; // x, y, visibility per keypoint
    static constexpr int KPT_START = 6; // keypoints start at row 6 (after 4 bbox + 2 class)
    static constexpr PixelFormat TARGET_FORMAT = PixelFormat::RGB;
};

inline void nms_merge_sorted_bboxes(
    std::vector<RuneFanBladeWithR>& objs,
    std::vector<int>& out_indices,
    float nms_threshold
) {
    out_indices.clear();
    const size_t n = objs.size();

    for (size_t i = 0; i < n; ++i) {
        RuneFanBladeWithR& a = objs[i];
        bool keep = true;
        for (int idx: out_indices) {
            RuneFanBladeWithR& b = objs[idx];
            const float iou = utils::rect_ioU(a.bbox, b.bbox);
            if (std::isnan(iou) || iou > nms_threshold) {
                keep = false;
                if (a.color == b.color && iou > MERGE_MIN_IOU
                    && std::abs(a.confidence - b.confidence) < MERGE_CONF_ERROR)
                {
                    b.tmp_points.push_back(a.points);
                }
                break;
            }
        }
        if (keep)
            out_indices.push_back(static_cast<int>(i));
    }
}

inline std::vector<RuneFanBladeWithR> topk_and_nms(std::vector<RuneFanBladeWithR>& objs) {
    std::sort(objs.begin(), objs.end(), [](const RuneFanBladeWithR& a, const RuneFanBladeWithR& b) {
        return a.confidence > b.confidence;
    });

    if (static_cast<int>(objs.size()) > TOP_K)
        objs.resize(static_cast<size_t>(TOP_K));

    std::vector<int> indices;
    nms_merge_sorted_bboxes(objs, indices, NMS_THRESHOLD);

    std::vector<RuneFanBladeWithR> result;
    result.reserve(indices.size());

    for (size_t i = 0; i < indices.size(); ++i) {
        result.push_back(std::move(objs[indices[i]]));
        auto& ro = result.back();
        if (ro.tmp_points.size() >= 1) {
            std::vector<cv::Point2f> accum(ro.points.size(), cv::Point2f(0.0f, 0.0f));
            std::vector<int> count(ro.points.size(), 0);
            for (const auto& pts: ro.tmp_points) {
                if (pts.size() != ro.points.size())
                    continue;
                for (size_t k = 0; k < pts.size(); ++k) {
                    accum[k] += pts[k];
                    count[k]++;
                }
            }
            for (size_t k = 0; k < ro.points.size(); ++k) {
                if (count[k] > 0) {
                    ro.points[k] = (ro.points[k] + accum[k]) / static_cast<float>(count[k] + 1);
                }
            }
            ro.tmp_points.clear();
        }
    }

    return result;
}
struct RuneInfer::Impl {
    struct Params {
        double conf_threshold = 0.1;
        Mode mode;
        int input_w;
        int input_h;
        bool use_norm;
        PixelFormat target_format;
        template<typename M>
        void set_mode() {
            input_w = M::INPUT_W;
            input_h = M::INPUT_H;
            use_norm = M::USE_NORM;
            target_format = M::TARGET_FORMAT;
        }
        void load(const YAML::Node& config) {
            auto mode_str = config["model_type"].as<std::string>();
            mode = modeFromString(mode_str);
            switch (mode) {
                case Mode::CB: {
                    set_mode<ModelTraits<Mode::CB>>();
                    break;
                }
            }
            conf_threshold = config["conf_threshold"].as<double>();
        }
    } params_;
    Impl(const YAML::Node& config) {
        params_.load(config);
    }

    [[nodiscard]] std::vector<RuneFanBladeWithR> process(const cv::Mat& output_buffer) const {
        std::vector<RuneFanBladeWithR> results;
        results = post_process(output_buffer);
        return topk_and_nms(results);
    }
    [[nodiscard]] std::vector<RuneFanBladeWithR> post_process(const cv::Mat& output_buffer) const {
        if (output_buffer.empty())
            return {};
        switch (params_.mode) {
            case Mode::CB:
                return post_processCB_impl(output_buffer);
        }
        return {};
    }
    std::vector<RuneFanBladeWithR> post_processCB_impl(const cv::Mat& out) const {
        std::vector<RuneFanBladeWithR> results;
        const int cols = out.cols;
        auto remap_keypoints = [](const std::vector<cv::Point2f>& raw) {
            // Raw 9-pt: [0]=R_center, [1,8]=bottom, [2,3]=right, [4,5]=top, [6,7]=left
            // Compute edge-pair midpoints → map to old 6-pt PnP corners
            const cv::Point2f top = (raw[4] + raw[5]) * 0.5f;
            const cv::Point2f left = (raw[6] + raw[7]) * 0.5f;
            const cv::Point2f bottom = (raw[1] + raw[8]) * 0.5f;
            const cv::Point2f right = (raw[2] + raw[3]) * 0.5f;

            std::vector<cv::Point2f> out(ModelTraits<Mode::CB>::REMAPPED_KPTS);
            out[0] = top; // top edge center
            out[1] = left; // left edge center
            out[2] = bottom; // bottom edge center
            out[3] = right; // right edge center
            out[4] = (top + left + bottom + right) * 0.25f; // blade center
            out[5] = raw[0]; // R center
            return out;
        };
        for (int i = 0; i < cols; ++i) {
            const float r_score = out.at<float>(4, i);
            const float b_score = out.at<float>(5, i);
            float conf = -1;
            bool is_r = false;
            if (r_score > b_score) {
                is_r = true;
                conf = r_score;
            } else {
                is_r = false;
                conf = b_score;
            }
            if (conf < params_.conf_threshold)
                continue;
            const float cx = out.at<float>(0, i);
            const float cy = out.at<float>(1, i);
            const float ow = out.at<float>(2, i);
            const float oh = out.at<float>(3, i);
            RuneFanBladeWithR rune;
            rune.confidence = conf;
            rune.color = is_r ? RuneColor::RED : RuneColor::BLUE;
            rune.bbox = cv::Rect2f(cx - ow / 2, cy - oh / 2, ow, oh);
            cv::Mat kpts = out.col(i).rowRange(
                ModelTraits<Mode::CB>::KPT_START,
                ModelTraits<Mode::CB>::KPT_START
                    + (ModelTraits<Mode::CB>::NUM_POINTS * ModelTraits<Mode::CB>::KPT_DIM)
            );
            std::vector<cv::Point2f> raw(ModelTraits<Mode::CB>::NUM_POINTS);
            for (int j = 0; j < ModelTraits<Mode::CB>::NUM_POINTS; ++j) {
                raw[j] = cv::Point2f(
                    kpts.at<float>(j * ModelTraits<Mode::CB>::KPT_DIM, 0),
                    kpts.at<float>(j * ModelTraits<Mode::CB>::KPT_DIM + 1, 0)
                );
            }
            rune.points = remap_keypoints(raw);
            // rune.points = raw;
            results.push_back(rune);
        }
        return topk_and_nms(results);
    }

    int input_w() const noexcept {
        return params_.input_w;
    }
    int input_h() const noexcept {
        return params_.input_h;
    }
    bool use_norm() const noexcept {
        return params_.use_norm;
    }
    PixelFormat target_format() const noexcept {
        return params_.target_format;
    }
};

RuneInfer::RuneInfer(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
RuneInfer::~RuneInfer() noexcept {
    _impl.reset();
}
[[nodiscard]] std::vector<RuneFanBladeWithR> RuneInfer::process(const cv::Mat& output_buffer
) const {
    return _impl->process(output_buffer);
}

int RuneInfer::input_w() const noexcept {
    return _impl->input_w();
}
int RuneInfer::input_h() const noexcept {
    return _impl->input_h();
}
bool RuneInfer::use_norm() const noexcept {
    return _impl->use_norm();
}
PixelFormat RuneInfer::target_format() const noexcept {
    return _impl->target_format();
}
} // namespace awakening::auto_buff
