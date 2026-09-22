#include "armor_infer.hpp"
#include "tasks/auto_aim/type.hpp"
#include <cstddef>
#include <memory>

namespace awakening::auto_aim {
static constexpr float MERGE_CONF_ERROR = 0.95f;
static constexpr float MERGE_MIN_IOU = 0.9f;
static constexpr float NMS_THRESHOLD = 0.35;
static constexpr int TOP_K = 128;
enum class Mode : int { TUP, RP, AT1, AT2 };
inline Mode modeFromString(const std::string& s) noexcept {
    std::string str = utils::to_upper(s);
    if (str == "TUP")
        return Mode::TUP;
    if (str == "RP")
        return Mode::RP;
    if (str == "AT1")
        return Mode::AT1;
    if (str == "AT2")
        return Mode::AT2;
    return Mode::TUP;
}
template<Mode M>
struct ModelTraits; // declare
// TUP
template<>
struct ModelTraits<Mode::TUP> {
    static constexpr int INPUT_W = 416;
    static constexpr int INPUT_H = 416;
    static constexpr int NUM_CLASSES = 8;
    static constexpr int NUM_COLORS = 4;
    static constexpr bool USE_NORM = false;
    static constexpr PixelFormat TARGET_FORMAT = PixelFormat::BGR;
    static constexpr std::array CLASSES = { ArmorClass::SENTRY,  ArmorClass::NO1,
                                            ArmorClass::NO2,     ArmorClass::NO3,
                                            ArmorClass::NO4,     ArmorClass::NO5,
                                            ArmorClass::OUTPOST, ArmorClass::BASE,
                                            ArmorClass::UNKNOWN };
    static constexpr std::array COLORS = { ArmorColor::BLUE,
                                           ArmorColor::RED,
                                           ArmorColor::PURPLE,
                                           ArmorColor::NONE };
};

// RP
template<>
struct ModelTraits<Mode::RP> {
    static constexpr int INPUT_W = 640;
    static constexpr int INPUT_H = 640;
    static constexpr int NUM_CLASSES = 9;
    static constexpr int NUM_COLORS = 4;
    static constexpr bool USE_NORM = true;
    static constexpr PixelFormat TARGET_FORMAT = PixelFormat::RGB;
    static constexpr std::array CLASSES = { ArmorClass::SENTRY,  ArmorClass::NO1,
                                            ArmorClass::NO2,     ArmorClass::NO3,
                                            ArmorClass::NO4,     ArmorClass::NO5,
                                            ArmorClass::OUTPOST, ArmorClass::BASE,
                                            ArmorClass::UNKNOWN };
    static constexpr std::array COLORS = { ArmorColor::BLUE,
                                           ArmorColor::RED,
                                           ArmorColor::PURPLE,
                                           ArmorColor::NONE };
};
struct ModelTraits_AT {
    static constexpr int NUM_CLASSES = 9;
    static constexpr int NUM_COLORS = 4;
    static constexpr int NUM_KPTS = 4;
    static constexpr bool USE_NORM = true;
    static constexpr PixelFormat TARGET_FORMAT = PixelFormat::RGB;
    static constexpr std::array CLASSES = { ArmorClass::SENTRY,  ArmorClass::NO1,
                                            ArmorClass::NO2,     ArmorClass::NO3,
                                            ArmorClass::NO4,     ArmorClass::NO5,
                                            ArmorClass::OUTPOST, ArmorClass::BASE,
                                            ArmorClass::UNKNOWN };
    static constexpr std::array COLORS = { ArmorColor::BLUE,
                                           ArmorColor::RED,
                                           ArmorColor::NONE,
                                           ArmorColor::PURPLE };
};
template<>
struct ModelTraits<Mode::AT1> {
    static constexpr int INPUT_W = 640;
    static constexpr int INPUT_H = 640;
    static constexpr int NUM_KPTS = ModelTraits_AT::NUM_KPTS;
    static constexpr bool USE_NORM = ModelTraits_AT::USE_NORM;
    static constexpr PixelFormat TARGET_FORMAT = ModelTraits_AT::TARGET_FORMAT;
    static constexpr std::array CLASSES = ModelTraits_AT::CLASSES;
    static constexpr std::array COLORS = ModelTraits_AT::COLORS;
};
template<>
struct ModelTraits<Mode::AT2> {
    static constexpr int INPUT_W = 768;
    static constexpr int INPUT_H = 576;
    static constexpr int NUM_KPTS = ModelTraits_AT::NUM_KPTS;
    static constexpr bool USE_NORM = ModelTraits_AT::USE_NORM;
    static constexpr PixelFormat TARGET_FORMAT = ModelTraits_AT::TARGET_FORMAT;
    static constexpr std::array CLASSES = ModelTraits_AT::CLASSES;
    static constexpr std::array COLORS = ModelTraits_AT::COLORS;
};

inline void nms_merge_sorted_bboxes(
    std::vector<Armor>& objs,
    std::vector<int>& out_indices,
    float nms_threshold
) {
    out_indices.clear();
    const size_t n = objs.size();

    for (size_t i = 0; i < n; ++i) {
        Armor& a = objs[i];
        bool keep = true;
        for (int idx: out_indices) {
            Armor& b = objs[idx];
            const float iou =
                utils::rect_ioU(a.net->key_points.bounding_box(), b.net->key_points.bounding_box());
            if (std::isnan(iou) || iou > nms_threshold) {
                keep = false;
                if (a.number == b.number && a.color == b.color && iou > MERGE_MIN_IOU
                    && std::abs(a.net->confidence - b.net->confidence) < MERGE_CONF_ERROR)
                {
                    // accumulate points for later averaging
                    b.net->tmp_points.push_back(a.net->key_points.points);
                }
                break;
            }
        }
        if (keep)
            out_indices.push_back(static_cast<int>(i));
    }
}

inline std::vector<Armor> topk_and_nms(std::vector<Armor>& objs) {
    std::sort(objs.begin(), objs.end(), [](const Armor& a, const Armor& b) {
        return a.net->confidence > b.net->confidence;
    });

    if (static_cast<int>(objs.size()) > TOP_K)
        objs.resize(static_cast<size_t>(TOP_K));

    std::vector<int> indices;
    nms_merge_sorted_bboxes(objs, indices, NMS_THRESHOLD);

    std::vector<Armor> result;
    result.reserve(indices.size());

    for (size_t i = 0; i < indices.size(); ++i) {
        result.push_back(std::move(objs[indices[i]]));
        auto& ro = result.back();
        if (ro.net->tmp_points.size() >= 1) {
            constexpr size_t N = ArmorKeyPointsIndex::N;
            std::array<cv::Point2f, N> accum {};
            std::array<int, N> count {};
            const auto& base_pts_opt = ro.net->key_points.points;
            for (size_t k = 0; k < N; ++k) {
                if (base_pts_opt[k]) {
                    accum[k] += *base_pts_opt[k];
                    count[k]++;
                }
            }
            for (const auto& pts_opt: ro.net->tmp_points) {
                for (size_t k = 0; k < N; ++k) {
                    if (pts_opt[k]) {
                        accum[k] += *pts_opt[k];
                        count[k]++;
                    }
                }
            }
            std::array<std::optional<cv::Point2f>, ArmorKeyPointsIndex::N> final_pts {};
            for (size_t k = 0; k < ArmorKeyPointsIndex::N; ++k) {
                if (count[k] > 0) {
                    if (base_pts_opt[k]) {
                        final_pts[k] = accum[k] / static_cast<float>(count[k]);
                    }
                }
            }
            ro.net->key_points.points = final_pts;
            ro.net->tmp_points.clear();
        }
    }

    return result;
}
struct ArmorInfer::Impl {
    struct Params {
        double conf_threshold = 0.1;
        Mode mode;
        int input_w;
        int input_h;
        bool use_norm;
        PixelFormat target_format;
        template<typename M>
        void setMode() {
            input_w = M::INPUT_W;
            input_h = M::INPUT_H;
            use_norm = M::USE_NORM;
            target_format = M::TARGET_FORMAT;
        }
        void load(const YAML::Node& config) {
            auto mode_str = config["model_type"].as<std::string>();
            mode = modeFromString(mode_str);
            switch (mode) {
                case Mode::TUP: {
                    setMode<ModelTraits<Mode::TUP>>();
                    break;
                }
                case Mode::RP: {
                    setMode<ModelTraits<Mode::RP>>();
                    break;
                }
                case Mode::AT1: {
                    setMode<ModelTraits<Mode::AT1>>();
                    break;
                }
                case Mode::AT2: {
                    setMode<ModelTraits<Mode::AT2>>();
                    break;
                }
            }
            conf_threshold = config["conf_threshold"].as<double>();
        }
    } params_;
    Impl(const YAML::Node& config) {
        params_.load(config);
    }

    [[nodiscard]] std::vector<Armor> process(const cv::Mat& output_buffer) const {
        std::vector<Armor> results;
        results = post_process(output_buffer);
        return results;
    }
    [[nodiscard]] std::vector<Armor> post_process(const cv::Mat& output_buffer) const {
        if (output_buffer.empty())
            return {};
        switch (params_.mode) {
            case Mode::TUP:
                return post_processTUP_impl(output_buffer);
            case Mode::RP:
                return postProcessRP_impl(output_buffer);
            case Mode::AT1:
                return post_processAT_impl(output_buffer);
            case Mode::AT2:
                return post_processAT_impl(output_buffer);
        }
        return {};
    }
    std::vector<Armor> post_processTUP_impl(const cv::Mat& out) const {
        struct GridAndStride {
            int grid0;
            int grid1;
            int stride;
        };
        static std::optional<std::vector<GridAndStride>> _grid_strides;
        if (!_grid_strides) {
            auto generate_grids_and_stride = [&]() {
                std::vector<GridAndStride> grid_strides;
                for (int stride: { 8, 16, 32 }) {
                    const int num_w = input_w() / stride;
                    const int num_h = input_h() / stride;
                    grid_strides.reserve(grid_strides.size() + num_w * num_h);
                    for (int gy = 0; gy < num_h; ++gy) {
                        for (int gx = 0; gx < num_w; ++gx) {
                            grid_strides.push_back(GridAndStride { gx, gy, stride });
                        }
                    }
                }
                return grid_strides;
            };
            _grid_strides = generate_grids_and_stride();
        }
        const auto& grid_strides = _grid_strides.value();
        std::vector<Armor> out_objs;
        const int num_anchors =
            static_cast<int>(std::min<size_t>(grid_strides.size(), static_cast<size_t>(out.rows)));
        using I = ArmorKeyPointsIndex;
        for (int a = 0; a < num_anchors; ++a) {
            const float confidence = out.at<float>(a, 8);
            if (confidence < params_.conf_threshold) {
                continue;
            }

            const auto& gs = grid_strides[a];
            const int gx = gs.grid0, gy = gs.grid1, stride = gs.stride;

            // color & class
            const int color_offset = 9;
            const int num_colors = ModelTraits<Mode::TUP>::NUM_COLORS;
            const int num_classes = ModelTraits<Mode::TUP>::NUM_CLASSES;

            cv::Mat color_scores = out.row(a).colRange(color_offset, color_offset + num_colors);
            cv::Mat class_scores = out.row(a).colRange(
                color_offset + num_colors,
                color_offset + num_colors + num_classes
            );

            double max_color, max_class;
            cv::Point color_id, class_id;
            cv::minMaxLoc(color_scores, nullptr, &max_color, nullptr, &color_id);
            cv::minMaxLoc(class_scores, nullptr, &max_class, nullptr, &class_id);

            const float x1 = (out.at<float>(a, 0) + gx) * stride;
            const float y1 = (out.at<float>(a, 1) + gy) * stride;
            const float x2 = (out.at<float>(a, 2) + gx) * stride;
            const float y2 = (out.at<float>(a, 3) + gy) * stride;
            const float x3 = (out.at<float>(a, 4) + gx) * stride;
            const float y3 = (out.at<float>(a, 5) + gy) * stride;
            const float x4 = (out.at<float>(a, 6) + gx) * stride;
            const float y4 = (out.at<float>(a, 7) + gy) * stride;

            Armor obj;
            auto& net = obj.net;
            net = Armor::NetCtx();
            net->color = ModelTraits<Mode::TUP>::COLORS[color_id.x];
            net->number = ModelTraits<Mode::TUP>::CLASSES[class_id.x];
            auto& key_points = net->key_points;
            key_points.points[std::to_underlying(I::LEFT_TOP)] = cv::Point2f(x1, y1);
            key_points.points[std::to_underlying(I::LEFT_BOTTOM)] = cv::Point2f(x2, y2);
            key_points.points[std::to_underlying(I::RIGHT_BOTTOM)] = cv::Point2f(x3, y3);
            key_points.points[std::to_underlying(I::RIGHT_TOP)] = cv::Point2f(x4, y4);
            net->confidence = confidence;
            out_objs.push_back(std::move(obj));
        }
        return topk_and_nms(out_objs);
    }
    std::vector<Armor> postProcessRP_impl(const cv::Mat& out) const {
        std::vector<Armor> out_objs;
        const int rows = out.rows;
        const int color_offset = 9;
        const int num_colors = ModelTraits<Mode::RP>::NUM_COLORS;
        const int num_classes = ModelTraits<Mode::RP>::NUM_CLASSES;
        using I = ArmorKeyPointsIndex;
        for (int r = 0; r < rows; ++r) {
            float conf_raw = out.at<float>(r, 8);
            const float confidence = static_cast<float>(utils::sigmoid(conf_raw));
            if (confidence < params_.conf_threshold)
                continue;
            cv::Mat color_scores = out.row(r).colRange(color_offset, color_offset + num_colors);
            cv::Mat class_scores = out.row(r).colRange(
                color_offset + num_colors,
                color_offset + num_colors + num_classes
            );

            double max_color_score, max_class_score;
            cv::Point color_id, class_id;
            cv::minMaxLoc(color_scores, nullptr, &max_color_score, nullptr, &color_id);
            cv::minMaxLoc(class_scores, nullptr, &max_class_score, nullptr, &class_id);
            const float x1 = out.at<float>(r, 0);
            const float y1 = out.at<float>(r, 1);
            const float x2 = out.at<float>(r, 2);
            const float y2 = out.at<float>(r, 3);
            const float x3 = out.at<float>(r, 4);
            const float y3 = out.at<float>(r, 5);
            const float x4 = out.at<float>(r, 6);
            const float y4 = out.at<float>(r, 7);
            Armor obj;
            auto& net = obj.net;
            net = Armor::NetCtx();
            net->color = ModelTraits<Mode::RP>::COLORS[color_id.x];

            net->number = ModelTraits<Mode::RP>::CLASSES[class_id.x];

            auto& key_points = net->key_points;
            key_points.points[I::LEFT_TOP] = cv::Point2f(x1, y1);
            key_points.points[I::LEFT_BOTTOM] = cv::Point2f(x2, y2);
            key_points.points[I::RIGHT_BOTTOM] = cv::Point2f(x3, y3);
            key_points.points[I::RIGHT_TOP] = cv::Point2f(x4, y4);
            net->confidence = confidence;

            out_objs.push_back(std::move(obj));
        }

        return topk_and_nms(out_objs);
    }
    std::vector<Armor> post_processAT_impl(const cv::Mat& out) const {
        std::vector<Armor> out_objs;

        constexpr int nkpt = ModelTraits_AT::NUM_KPTS;
        constexpr int nk = nkpt * 2; // keypoints flattened
        constexpr int conf_offset = 4;
        constexpr int cls_offset = 5;
        constexpr int color_offset = 6;
        constexpr int kpt_offset = color_offset + ModelTraits_AT::NUM_COLORS;
        constexpr int min_det_dim = kpt_offset + nk;

        const int max_det = out.rows;
        const int det_dim = out.cols;
        if (det_dim < min_det_dim) {
            return {};
        }

        using I = ArmorKeyPointsIndex;
        for (int i = 0; i < max_det; ++i) {
            const float* row = out.ptr<float>(i);
            float conf = row[conf_offset];
            if (!std::isfinite(conf) || conf < params_.conf_threshold)
                continue;

            const int cls_id = static_cast<int>(row[cls_offset]);
            if (cls_id < 0 || cls_id >= ModelTraits_AT::NUM_CLASSES) {
                continue;
            }

            int color_id = 0;
            float color_score = row[color_offset];
            for (int c = 1; c < ModelTraits_AT::NUM_COLORS; ++c) {
                const float score = row[color_offset + c];
                if (score > color_score) {
                    color_score = score;
                    color_id = c;
                }
            }

            Armor obj;
            auto& net = obj.net;
            net = Armor::NetCtx();

            net->confidence = conf;
            net->color = ModelTraits_AT::COLORS[color_id];
            net->number = ModelTraits_AT::CLASSES[cls_id];
            auto getKeyPoints = [&](int k) {
                float kx = row[kpt_offset + 2 * k];
                float ky = row[kpt_offset + 2 * k + 1];
                return cv::Point2f(kx, ky);
            };
            auto& key_points = net->key_points;
            // key_points.points[std::to_underlying(I::LEFT_TOP)] = getKeyPoints(0);
            // key_points.points[std::to_underlying(I::LEFT_BOTTOM)] = getKeyPoints(1);
            // key_points.points[std::to_underlying(I::RIGHT_BOTTOM)] = getKeyPoints(2);
            // key_points.points[std::to_underlying(I::RIGHT_TOP)] = getKeyPoints(3);
            std::vector<cv::Point2f> pts;
            for (int k = 0; k < nkpt; ++k)
                pts.push_back(getKeyPoints(k));
            std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.y < b.y;
            });
            std::vector<cv::Point2f> top_pts = std::vector<cv::Point2f> { pts[0], pts[1] };
            std::vector<cv::Point2f> bottom_pts = std::vector<cv::Point2f> { pts[2], pts[3] };

            if (top_pts[0].x > top_pts[1].x)
                std::swap(top_pts[0], top_pts[1]);
            if (bottom_pts[0].x > bottom_pts[1].x)
                std::swap(bottom_pts[0], bottom_pts[1]);

            key_points.points[std::to_underlying(I::LEFT_TOP)] = top_pts[0];
            key_points.points[std::to_underlying(I::RIGHT_TOP)] = top_pts[1];
            key_points.points[std::to_underlying(I::LEFT_BOTTOM)] = bottom_pts[0];
            key_points.points[std::to_underlying(I::RIGHT_BOTTOM)] = bottom_pts[1];
            out_objs.emplace_back(std::move(obj));
        }

        return out_objs;
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

ArmorInfer::ArmorInfer(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
ArmorInfer::~ArmorInfer() noexcept {
    _impl.reset();
}
[[nodiscard]] std::vector<Armor> ArmorInfer::process(const cv::Mat& output_buffer) const {
    return _impl->process(output_buffer);
}

int ArmorInfer::input_w() const noexcept {
    return _impl->input_w();
}
int ArmorInfer::input_h() const noexcept {
    return _impl->input_h();
}
bool ArmorInfer::use_norm() const noexcept {
    return _impl->use_norm();
}
PixelFormat ArmorInfer::target_format() const noexcept {
    return _impl->target_format();
}
} // namespace awakening::auto_aim
