#pragma once

#include "tasks/vslam/frame.hpp"
#include "utils/net_detector/net_detector_base.hpp"
#ifdef USE_OPENVINO
    #include "utils/net_detector/openvino/net_detector_openvino.hpp"
#endif
#ifdef USE_TRT
    #include "utils/net_detector/tensorrt/net_detector_tensorrt.hpp"
#endif
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/node/node.h>

namespace awakening::vslam {
class Gcn2 {
public:
    struct Params {
        std::string model_path;
        std::string backend = "openvino";
        int input_width = 640;
        int input_height = 480;
        int border = 16;
        int dist_thresh = 8;
        float score_threshold = 0.05F;
        int max_features = 0;
        double match_threshold = 20.0;

        void load(const YAML::Node& config) {
            if (config["model_path"]) {
                model_path = config["model_path"].as<std::string>();
            } else {
                model_path = "/home/hy/paper_work/GCNv2_SLAM/GCN2/gcn2_640x480.onnx";
            }
            if (config["backend"]) {
                backend = config["backend"].as<std::string>();
            }
            std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            if (config["input_width"]) {
                input_width = config["input_width"].as<int>();
            }
            if (config["input_height"]) {
                input_height = config["input_height"].as<int>();
            }
            if (config["border"]) {
                border = config["border"].as<int>();
            }
            if (config["dist_thresh"]) {
                dist_thresh = config["dist_thresh"].as<int>();
            }
            if (config["score_threshold"]) {
                score_threshold = config["score_threshold"].as<float>();
            }
            if (config["max_features"]) {
                max_features = config["max_features"].as<int>();
            } else if (config["nfeatures"]) {
                max_features = config["nfeatures"].as<int>();
            }
            if (config["match_threshold"]) {
                match_threshold = config["match_threshold"].as<double>();
            }
        }
    } params_;

    explicit Gcn2(const YAML::Node& config) {
        params_.load(config);
        if (params_.input_width <= 0 || params_.input_height <= 0) {
            throw std::runtime_error("Gcn2 input size must be positive");
        }
        if (has_extension(params_.model_path, ".pt")) {
            throw std::runtime_error(
                "Gcn2 uses the project NetDetector backends; convert the TorchScript .pt model to ONNX "
                "and set gcn2.model_path to the .onnx file"
            );
        }

        utils::NetDetectorBase::Config net_cfg {
            .target_format = PixelFormat::GRAY,
            .preprocess_scale = 1.0 / 255.0,
            .target_w = params_.input_width,
            .target_h = params_.input_height,
        };
        bool backend_valid = false;
#ifdef USE_OPENVINO
        if (params_.backend == "openvino") {
            YAML::Node ov_config = config["net_detector"] && config["net_detector"]["openvino"] ?
                config["net_detector"]["openvino"] :
                YAML::Node {};
            ov_config["model_path"] = params_.model_path;
            if (!ov_config["device_name"]) {
                ov_config["device_name"] = "CPU";
            }
            if (!ov_config["infer_request_buffer_num"]) {
                ov_config["infer_request_buffer_num"] = 1;
            }
            net_detector_ = std::make_unique<utils::NetDetectorOpenVINO>(ov_config, net_cfg);
            backend_valid = true;
        }
#endif
#ifdef USE_TRT
        if (params_.backend == "tensorrt") {
            YAML::Node trt_config = config["net_detector"] && config["net_detector"]["tensorrt"] ?
                config["net_detector"]["tensorrt"] :
                YAML::Node {};
            trt_config["model_path"] = params_.model_path;
            if (!trt_config["copy_context_num"]) {
                trt_config["copy_context_num"] = 1;
            }
            if (!trt_config["min_free_mem_ratio"]) {
                trt_config["min_free_mem_ratio"] = 0.1;
            }
            trt_config["use_cuda_preproces"] = false;
            net_detector_ = std::make_unique<utils::NetDetectorTensorrt>(trt_config, net_cfg);
            backend_valid = true;
        }
#endif
        if (!backend_valid) {
            throw std::runtime_error("Invalid Gcn2 backend: " + params_.backend);
        }
    }

    void detect(Frame& frame) {
        frame.keypoints.clear();
        frame.descriptors.release();
        frame.detected = false;

        if (frame.img_gray.empty()) {
            return;
        }

        cv::Mat gray = ensure_gray(frame.img_gray);
        auto net_output = net_detector_->detect(gray, PixelFormat::GRAY);
        std::vector<cv::Mat> outputs = net_output.outputs;
        if (outputs.size() != 2) {
            throw std::runtime_error("GCN2 model must return two outputs");
        }

        cv::Mat pts;
        cv::Mat descriptors;
        float ratio_width = static_cast<float>(gray.cols) / static_cast<float>(params_.input_width);
        float ratio_height = static_cast<float>(gray.rows) / static_cast<float>(params_.input_height);
        DenseOutputs dense_outputs;
        if (try_split_outputs(outputs[0], outputs[1], pts, descriptors)) {
            nms(pts, descriptors, frame.keypoints, frame.descriptors, ratio_width, ratio_height);
        } else if (try_dense_outputs(outputs[0], outputs[1], dense_outputs)) {
            dense_to_features(
                dense_outputs,
                frame.keypoints,
                frame.descriptors,
                ratio_width,
                ratio_height
            );
        } else {
            throw std::runtime_error("Unsupported GCN2 output shape");
        }
        frame.detected = true;
    }

    utils::NetDetectorBase::Ptr net_detector_;

private:
    struct Candidate {
        int index = 0;
        int u = 0;
        int v = 0;
        float score = 0.0F;
    };

    struct DenseOutputs {
        cv::Mat desc;
        cv::Mat det;
        int desc_width = 0;
        int desc_height = 0;
        int det_width = 0;
        int det_height = 0;
        bool det_is_cells = false;
    };

    static bool has_extension(const std::string& path, const std::string& ext) {
        std::filesystem::path fs_path(path);
        std::string actual = fs_path.extension().string();
        std::transform(actual.begin(), actual.end(), actual.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return actual == ext;
    }

    static cv::Mat ensure_gray(const cv::Mat& image) {
        if (image.channels() == 1) {
            return image;
        }
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }

    static cv::Mat as_rows(const cv::Mat& output, int cols) {
        cv::Mat continuous = output.isContinuous() ? output : output.clone();
        int total = static_cast<int>(continuous.total());
        if (cols <= 0 || total % cols != 0) {
            throw std::runtime_error("Unexpected GCN2 output shape");
        }
        return continuous.reshape(1, total / cols);
    }

    static bool try_split_outputs(
        const cv::Mat& maybe_pts,
        const cv::Mat& maybe_descriptors,
        cv::Mat& pts,
        cv::Mat& descriptors
    ) {
        try {
            cv::Mat candidate_pts = as_rows(maybe_pts, 3);
            cv::Mat candidate_descriptors = as_rows(maybe_descriptors, 32);
            if (candidate_pts.rows == candidate_descriptors.rows) {
                pts = candidate_pts;
                descriptors = candidate_descriptors;
                return true;
            }
        } catch (const std::runtime_error&) {
            return false;
        }
        return false;
    }

    static bool is_dense_descriptor(const cv::Mat& output) {
        return (output.dims == 4 && output.size[0] == 1 && output.size[1] == 256 &&
                   output.size[2] > 0 && output.size[3] > 0)
            || (output.dims == 2 && output.rows % 256 == 0 && output.cols > 0);
    }

    bool is_dense_detector_image(const cv::Mat& output) const {
        return (output.dims == 4 && output.size[0] == 1 && output.size[1] == 1 &&
                   output.size[2] == params_.input_height && output.size[3] == params_.input_width)
            || (output.dims == 2 && output.rows == params_.input_height &&
                   output.cols == params_.input_width);
    }

    static bool is_dense_detector_cells(const cv::Mat& output, const cv::Mat& descriptor) {
        if (output.dims == 4 && descriptor.dims == 4) {
            return output.size[0] == 1 && output.size[1] == 256 &&
                output.size[2] == descriptor.size[2] && output.size[3] == descriptor.size[3];
        }
        return output.dims == 2 && descriptor.dims == 2 && output.rows == descriptor.rows &&
            output.cols == descriptor.cols && output.rows % 256 == 0;
    }

    bool try_dense_outputs(const cv::Mat& first, const cv::Mat& second, DenseOutputs& outputs) const {
        const cv::Mat* desc = nullptr;
        const cv::Mat* det = nullptr;
        bool det_is_cells = false;
        if (is_dense_descriptor(first) && is_dense_detector_image(second)) {
            desc = &first;
            det = &second;
        } else if (is_dense_descriptor(second) && is_dense_detector_image(first)) {
            desc = &second;
            det = &first;
        } else if (is_dense_descriptor(first) && is_dense_detector_cells(second, first)) {
            desc = &first;
            det = &second;
            det_is_cells = true;
        } else if (is_dense_descriptor(second) && is_dense_detector_cells(first, second)) {
            desc = &second;
            det = &first;
            det_is_cells = true;
        } else {
            return false;
        }

        outputs.desc = *desc;
        outputs.det = *det;
        dense_dims(*desc, outputs.desc_width, outputs.desc_height);
        dense_dims(*det, outputs.det_width, outputs.det_height);
        outputs.det_is_cells = det_is_cells;
        return true;
    }

    static void dense_dims(const cv::Mat& output, int& width, int& height) {
        if (output.dims == 4) {
            width = output.size[3];
            height = output.size[2];
            return;
        }
        width = output.cols;
        height = output.rows / 256;
    }

    static float mat_float_at(const cv::Mat& mat, int row, int col) {
        switch (mat.depth()) {
            case CV_32F:
                return mat.at<float>(row, col);
            case CV_64F:
                return static_cast<float>(mat.at<double>(row, col));
            case CV_32S:
                return static_cast<float>(mat.at<int>(row, col));
            case CV_16S:
                return static_cast<float>(mat.at<short>(row, col));
            case CV_8U:
                return static_cast<float>(mat.at<unsigned char>(row, col));
            default:
                throw std::runtime_error("Unsupported GCN2 pts output type");
        }
    }

    static unsigned char descriptor_byte_at(const cv::Mat& mat, int row, int col) {
        switch (mat.depth()) {
            case CV_8U:
                return mat.at<unsigned char>(row, col);
            case CV_32F:
                return static_cast<unsigned char>(std::clamp(std::lround(mat.at<float>(row, col)), 0L, 255L));
            case CV_64F:
                return static_cast<unsigned char>(std::clamp(std::lround(mat.at<double>(row, col)), 0L, 255L));
            case CV_32S:
                return static_cast<unsigned char>(std::clamp(mat.at<int>(row, col), 0, 255));
            default:
                throw std::runtime_error("Unsupported GCN2 descriptor output type");
        }
    }

    static const float* float_data(const cv::Mat& mat, cv::Mat& storage) {
        if (mat.depth() == CV_32F && mat.isContinuous()) {
            return mat.ptr<float>();
        }
        mat.convertTo(storage, CV_32F);
        if (!storage.isContinuous()) {
            storage = storage.clone();
        }
        return storage.ptr<float>();
    }

    static float bilinear_sample(
        const float* desc,
        int channel,
        float x,
        float y,
        int width,
        int height
    ) {
        int x0 = static_cast<int>(std::floor(x));
        int y0 = static_cast<int>(std::floor(y));
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        float wx = x - static_cast<float>(x0);
        float wy = y - static_cast<float>(y0);

        auto at = [&](int yy, int xx) {
            if (xx < 0 || xx >= width || yy < 0 || yy >= height) {
                return 0.0F;
            }
            return desc[(channel * height + yy) * width + xx];
        };

        float v00 = at(y0, x0);
        float v01 = at(y0, x1);
        float v10 = at(y1, x0);
        float v11 = at(y1, x1);
        return (1.0F - wx) * (1.0F - wy) * v00 + wx * (1.0F - wy) * v01 +
            (1.0F - wx) * wy * v10 + wx * wy * v11;
    }

    static float detector_score_at(const DenseOutputs& outputs, const float* det, int u, int v) {
        if (!outputs.det_is_cells) {
            return det[v * outputs.det_width + u];
        }
        constexpr int upscale = 16;
        int cell_x = u / upscale;
        int cell_y = v / upscale;
        int offset_x = u % upscale;
        int offset_y = v % upscale;
        int channel = offset_y * upscale + offset_x;
        return det[(channel * outputs.det_height + cell_y) * outputs.det_width + cell_x];
    }

    std::vector<Candidate> select_candidates(const std::vector<Candidate>& candidates) const {
        cv::Mat grid(params_.input_height, params_.input_width, CV_8UC1, cv::Scalar(0));
        std::vector<Candidate> selected;
        selected.reserve(candidates.size());
        for (const Candidate& candidate: candidates) {
            if (candidate.u < params_.border || candidate.v < params_.border ||
                candidate.u >= params_.input_width - params_.border ||
                candidate.v >= params_.input_height - params_.border)
            {
                continue;
            }
            if (grid.at<unsigned char>(candidate.v, candidate.u) != 0) {
                continue;
            }

            selected.push_back(candidate);
            int u0 = std::max(0, candidate.u - params_.dist_thresh);
            int u1 = std::min(params_.input_width - 1, candidate.u + params_.dist_thresh);
            int v0 = std::max(0, candidate.v - params_.dist_thresh);
            int v1 = std::min(params_.input_height - 1, candidate.v + params_.dist_thresh);
            grid(cv::Range(v0, v1 + 1), cv::Range(u0, u1 + 1)).setTo(1);

            if (params_.max_features > 0 && static_cast<int>(selected.size()) >= params_.max_features) {
                break;
            }
        }
        return selected;
    }

    void dense_to_features(
        const DenseOutputs& outputs,
        std::vector<cv::KeyPoint>& keypoints,
        cv::Mat& descriptors,
        float ratio_width,
        float ratio_height
    ) const {
        cv::Mat det_storage;
        cv::Mat desc_storage;
        const float* det = float_data(outputs.det, det_storage);
        const float* desc = float_data(outputs.desc, desc_storage);

        std::vector<Candidate> candidates;
        candidates.reserve(params_.input_width * params_.input_height / 16);
        for (int v = 0; v < params_.input_height; ++v) {
            for (int u = 0; u < params_.input_width; ++u) {
                float score = detector_score_at(outputs, det, u, v);
                if (score >= params_.score_threshold) {
                    candidates.push_back(Candidate {
                        .index = v * params_.input_width + u,
                        .u = u,
                        .v = v,
                        .score = score,
                    });
                }
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            return lhs.score > rhs.score;
        });
        std::vector<Candidate> selected = select_candidates(candidates);

        keypoints.reserve(keypoints.size() + selected.size());
        descriptors.create(static_cast<int>(selected.size()), 32, CV_8U);
        for (int i = 0; i < static_cast<int>(selected.size()); ++i) {
            const Candidate& candidate = selected[i];
            keypoints.emplace_back(
                static_cast<float>(candidate.u) * ratio_width,
                static_cast<float>(candidate.v) * ratio_height,
                1.0F,
                -1.0F,
                candidate.score
            );

            float sample_x = static_cast<float>(candidate.u) *
                static_cast<float>(outputs.desc_width - 1) / static_cast<float>(params_.input_width);
            float sample_y = static_cast<float>(candidate.v) *
                static_cast<float>(outputs.desc_height - 1) / static_cast<float>(params_.input_height);
            for (int byte_idx = 0; byte_idx < 32; ++byte_idx) {
                unsigned char packed = 0;
                for (int bit = 0; bit < 8; ++bit) {
                    int channel = byte_idx * 8 + bit;
                    float value = bilinear_sample(
                        desc,
                        channel,
                        sample_x,
                        sample_y,
                        outputs.desc_width,
                        outputs.desc_height
                    );
                    if (value > 0.0F) {
                        packed |= static_cast<unsigned char>(1U << bit);
                    }
                }
                descriptors.at<unsigned char>(i, byte_idx) = packed;
            }
        }
    }

    void nms(
        const cv::Mat& pts,
        const cv::Mat& desc,
        std::vector<cv::KeyPoint>& keypoints,
        cv::Mat& descriptors,
        float ratio_width,
        float ratio_height
    ) const {
        std::vector<Candidate> candidates;
        candidates.reserve(pts.rows);
        for (int i = 0; i < pts.rows; ++i) {
            int u = cvRound(mat_float_at(pts, i, 0));
            int v = cvRound(mat_float_at(pts, i, 1));
            float score = mat_float_at(pts, i, 2);
            if (score < params_.score_threshold || u < 0 || v < 0 || u >= params_.input_width ||
                v >= params_.input_height)
            {
                continue;
            }
            candidates.push_back(Candidate {.index = i, .u = u, .v = v, .score = score});
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            return lhs.score > rhs.score;
        });

        std::vector<Candidate> selected = select_candidates(candidates);

        keypoints.reserve(keypoints.size() + selected.size());
        descriptors.create(static_cast<int>(selected.size()), 32, CV_8U);
        for (int i = 0; i < static_cast<int>(selected.size()); ++i) {
            const Candidate& candidate = selected[i];
            keypoints.emplace_back(
                static_cast<float>(candidate.u) * ratio_width,
                static_cast<float>(candidate.v) * ratio_height,
                1.0F,
                -1.0F,
                candidate.score
            );
            for (int j = 0; j < 32; ++j) {
                descriptors.at<unsigned char>(i, j) = descriptor_byte_at(desc, candidate.index, j);
            }
        }
    }
};
} // namespace awakening::vslam
