#include <cstddef>
#ifdef USE_TRT
    #include "net_detector_tensorrt.hpp"
    #include "utils/buffer.hpp"
    #include "utils/common/image.hpp"
    #include "utils/cuda/letter_box.hpp"
    #include "utils/logger.hpp"
    #include <NvOnnxParser.h>
    #include <array>
    #include <cuda_runtime.h>
    #include <fstream>
    #include <memory>
    #include <opencv2/core/hal/interface.h>
    #include <opencv2/dnn.hpp>
    #include <opencv2/highgui.hpp>
    #include <string>
    #include <vector>
namespace awakening::utils {
    #define TRT_CHECK(expr) \
        do { \
            const auto _ret = (expr); \
            if (_ret != cudaSuccess) { \
                std::cerr << "\033[31mCUDA error: " << cudaGetErrorString(_ret) << " (" << #expr \
                          << ")\033[0m\n"; \
                std::abort(); \
            } \
        } while (0)
struct NetDetectorTensorrt::Impl {
    struct Params {
        std::string model_path;
        int copy_context_num = 1;
        double min_free_mem_ratio = 0.1;
        bool use_cuda_preproces = true;
        void load(const YAML::Node& config) {
            model_path = replace_root_dir(config["model_path"].as<std::string>());
            copy_context_num = config["copy_context_num"].as<int>();
            min_free_mem_ratio = config["min_free_mem_ratio"].as<double>();
            use_cuda_preproces = config["use_cuda_preproces"].as<bool>();
        }
    } params_;
    class TRTLogger: public nvinfer1::ILogger {
    public:
        explicit TRTLogger(
            nvinfer1::ILogger::Severity severity = nvinfer1::ILogger::Severity::kWARNING
        ):
            severity_(severity) {}
        void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override {
            if (severity <= severity_) {
                std::cerr << msg << std::endl;
            }
        }
        nvinfer1::ILogger::Severity severity_;
    };
    static int64_t volume(const nvinfer1::Dims& dims) {
        int64_t v = 1;
        for (int i = 0; i < dims.nbDims; ++i)
            v *= dims.d[i];
        return v;
    }
    Impl(const YAML::Node& config, Config c) {
        config_ = c;

        params_.load(config);
        buildEngine(params_.model_path);
        auto tmp_ctx =
            std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());

        if (!tmp_ctx)
            throw std::runtime_error("Failed to create execution context");
        const int num_io_tensors = engine_->getNbIOTensors();
        for (int i = 0; i < num_io_tensors; ++i) {
            const char* name = engine_->getIOTensorName(i);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
                input_name_ = name;
            } else {
                output_names_.push_back(name);
            }
        }
        if (input_name_ == nullptr || output_names_.empty()) {
            throw std::runtime_error("Invalid TensorRT IO tensors");
        }
        input_dims_ = nvinfer1::Dims4 { 1,
                                        config_.target_format == PixelFormat::GRAY ? 1 : 3,
                                        config_.target_h,
                                        config_.target_w };
        tmp_ctx->setInputShape(input_name_, input_dims_);
        auto dims = tmp_ctx->getTensorShape(input_name_);
        if (dims.nbDims == -1) {
            throw std::runtime_error("Input shape not specified");
        }

        input_dims_ = dims;
        output_dims_.clear();
        output_szs_.clear();
        output_dims_.reserve(output_names_.size());
        output_szs_.reserve(output_names_.size());
        for (const char* output_name: output_names_) {
            auto output_dims = tmp_ctx->getTensorShape(output_name);
            if (output_dims.nbDims == -1) {
                throw std::runtime_error("Output shape not specified");
            }
            output_dims_.push_back(output_dims);
            output_szs_.push_back(static_cast<size_t>(volume(output_dims)));
        }
        tmp_ctx.reset();
        input_sz_ = volume(input_dims_);
        if (params_.copy_context_num < 1) {
            params_.copy_context_num = 1;
        }
        for (int i = 0; i < params_.copy_context_num; ++i) {
            if (i > 0) {
                size_t free_mem, total_mem;
                cudaMemGetInfo(&free_mem, &total_mem);

                AWAKENING_DEBUG("Free GPU memory: {} MB", free_mem / 1024.0 / 1024.0);
                AWAKENING_DEBUG("Total GPU memory: {} MB", total_mem / 1024.0 / 1024.0);
                double free_mem_ratio =
                    static_cast<double>(free_mem) / static_cast<double>(total_mem);
                if (free_mem_ratio < params_.min_free_mem_ratio && i > 0) {
                    AWAKENING_WARN(
                        "GPU memory is not enough! Free GPU memory: {:.2f}%",
                        free_mem_ratio * 100
                    );
                    break;
                }
            }

            Ctx ctx;
            ctx.context.reset(engine_->createExecutionContext());
            if (params_.use_cuda_preproces)
                ctx.letter_box = std::make_shared<__cuda::LetterBox>(config_);
            TRT_CHECK(cudaMalloc(&ctx.input_device_buffer, input_sz_ * sizeof(float)));
            ctx.output_device_buffers.resize(output_szs_.size(), nullptr);
            ctx.output_buffers.resize(output_szs_.size());
            for (size_t output_idx = 0; output_idx < output_szs_.size(); ++output_idx) {
                TRT_CHECK(cudaMalloc(
                    &ctx.output_device_buffers[output_idx],
                    output_szs_[output_idx] * sizeof(float)
                ));
                ctx.output_buffers[output_idx].resize(output_szs_[output_idx]);
            }
            TRT_CHECK(cudaStreamCreate(&ctx.stream));
            ctx_buffers_.add_resource(std::move(ctx));
        }
    }
    void buildEngine(const std::string& onnx_path) {
        const std::string engine_path =
            onnx_path.substr(0, onnx_path.find_last_of('.')) + ".engine";

        runtime_.reset(nvinfer1::createInferRuntime(g_logger_));

        {
            std::ifstream fin(engine_path, std::ios::binary);
            if (fin.good()) {
                fin.seekg(0, std::ios::end);
                const size_t size = fin.tellg();
                fin.seekg(0, std::ios::beg);

                std::vector<char> data(size);
                fin.read(data.data(), size);

                engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
                if (engine_) {
                    AWAKENING_INFO("Loaded TensorRT engine: {}", engine_path);
                    return;
                }
                AWAKENING_ERROR("Failed to load TensorRT engine: {}", engine_path);
            }
        }

        AWAKENING_INFO("Building TensorRT engine from ONNX...");
        auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(g_logger_));

        auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(0));

        auto parser =
            std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, g_logger_));

        if (!parser->parseFromFile(
                onnx_path.c_str(),
                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)
            ))
        {
            AWAKENING_ERROR("Failed to parse ONNX: {}", onnx_path);
            throw std::runtime_error("Failed to parse ONNX: " + onnx_path);
        }

        auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);
        auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
            builder->buildSerializedNetwork(*network, *config)
        );
        auto profile = builder->createOptimizationProfile();

        int c = config_.target_format == PixelFormat::GRAY ? 1 : 3;
        nvinfer1::Dims4 dims { 1, c, config_.target_h, config_.target_w };

        const char* input_name = network->getInput(0)->getName();

        profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMIN, dims);
        profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kOPT, dims);
        profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMAX, dims);

        config->addOptimizationProfile(profile);

        engine_.reset(runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));

        if (!engine_)
            throw std::runtime_error("Engine build failed");

        std::ofstream fout(engine_path, std::ios::binary);
        fout.write(static_cast<const char*>(serialized->data()), serialized->size());

        AWAKENING_INFO("Engine built & saved: {}", engine_path);
    }

    static cv::Mat output_to_mat(const nvinfer1::Dims& dims, float* data) {
        if (dims.nbDims == 1) {
            return cv::Mat(dims.d[0], 1, CV_32F, data).clone();
        }
        if (dims.nbDims == 2) {
            return cv::Mat(dims.d[0], dims.d[1], CV_32F, data).clone();
        }
        if (dims.nbDims == 3) {
            return cv::Mat(dims.d[0] * dims.d[1], dims.d[2], CV_32F, data).clone();
        }
        if (dims.nbDims == 4) {
            int rows = dims.d[0] == 1 ? dims.d[1] * dims.d[2] :
                dims.d[0] * dims.d[1] * dims.d[2];
            return cv::Mat(rows, dims.d[3], CV_32F, data).clone();
        }
        return {};
    }

    OutPut detect(const cv::Mat& img, PixelFormat format) noexcept {
        if (img.empty()) {
            return {};
        }
        OutPut output;
        cv::Mat blob;
        if (!params_.use_cuda_preproces) { // 最大化ctx利用率,该部分不需要ctx则暂时不请求c
            output.resized_img =
                utils::letterbox(img, output.transform_matrix, config_.target_w, config_.target_h);
            auto swap_rb = format != config_.target_format;
            blob = cv::dnn::blobFromImage(
                output.resized_img,
                config_.preprocess_scale,
                cv::Size(config_.target_w, config_.target_h),
                cv::Scalar(0, 0, 0),
                swap_rb
            );
        }
        {
            auto r = ctx_buffers_.acquire();
            if (!r) {
                return output;
            }
            auto& ctx = *r;
            void* input_device_buffer = ctx.input_device_buffer;
            if (params_.use_cuda_preproces) {
                auto tensor = ctx.letter_box->letterbox_pitched(
                    img.data,
                    format,
                    img.cols,
                    img.rows,
                    img.step,
                    output.transform_matrix,
                    ctx.stream
                );
                if (!tensor) {
                    return output;
                }
                input_device_buffer = tensor;
                output.resized_img = ctx.letter_box->tensor_to_mat(
                    static_cast<float*>(input_device_buffer),
                    ctx.stream,
                    format != config_.target_format
                );
            } else {
                TRT_CHECK(cudaMemcpyAsync(
                    ctx.input_device_buffer,
                    blob.ptr<float>(),
                    input_sz_ * sizeof(float),
                    cudaMemcpyHostToDevice,
                    ctx.stream
                ));
            }

            ctx.context->setTensorAddress(input_name_, input_device_buffer);
            for (size_t output_idx = 0; output_idx < output_names_.size(); ++output_idx) {
                ctx.context->setTensorAddress(
                    output_names_[output_idx],
                    ctx.output_device_buffers[output_idx]
                );
            }

            if (!ctx.context->enqueueV3(ctx.stream)) {
                AWAKENING_ERROR("enqueueV3 failed");
                return output;
            }

            for (size_t output_idx = 0; output_idx < output_names_.size(); ++output_idx) {
                TRT_CHECK(cudaMemcpyAsync(
                    ctx.output_buffers[output_idx].data(),
                    ctx.output_device_buffers[output_idx],
                    output_szs_[output_idx] * sizeof(float),
                    cudaMemcpyDeviceToHost,
                    ctx.stream
                ));
            }

            cudaStreamSynchronize(ctx.stream);

            output.outputs.reserve(output_names_.size());
            for (size_t output_idx = 0; output_idx < output_names_.size(); ++output_idx) {
                cv::Mat mat = output_to_mat(
                    output_dims_[output_idx],
                    ctx.output_buffers[output_idx].data()
                );
                if (!mat.empty()) {
                    output.outputs.push_back(std::move(mat));
                }
            }
        }

        return output;
    }
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    TRTLogger g_logger_;
    struct Ctx {
        std::shared_ptr<nvinfer1::IExecutionContext> context;
        void* input_device_buffer = nullptr;
        std::vector<void*> output_device_buffers;
        std::vector<std::vector<float>> output_buffers;
        cudaStream_t stream { nullptr };
        __cuda::LetterBox::Ptr letter_box;
    };
    ResourcePool<Ctx> ctx_buffers_;

    size_t input_sz_ { 0 };

    nvinfer1::Dims input_dims_ {};
    std::vector<nvinfer1::Dims> output_dims_;
    std::vector<size_t> output_szs_;
    const char* input_name_ { nullptr };
    std::vector<const char*> output_names_;
    Config config_;
};
NetDetectorTensorrt::NetDetectorTensorrt(const YAML::Node& config, Config c) {
    _impl = std::make_unique<NetDetectorTensorrt::Impl>(config, c);
}
NetDetectorTensorrt::~NetDetectorTensorrt() noexcept {
    _impl.reset();
}
NetDetectorTensorrt::OutPut
NetDetectorTensorrt::detect(const cv::Mat& img, PixelFormat format) noexcept {
    return _impl->detect(img, format);
}
} // namespace awakening::utils
#endif
