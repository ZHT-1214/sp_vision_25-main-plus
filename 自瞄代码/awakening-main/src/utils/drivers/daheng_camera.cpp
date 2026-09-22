#ifdef USE_DahengSDK
    #include "daheng_camera.hpp"

    #ifdef USE_TRT
        #include "utils/cuda/cvtcolor.hpp"
    #endif

namespace awakening {

void DahengCamera::init() {
    load(config_);
}

void DahengCamera::load(const YAML::Node& config) {
    std::string target_sn = config["target_sn"].as<std::string>("");
    if (!initialize_camera(target_sn)) {
        AWAKENING_ERROR("Failed to initialize Daheng camera with SN: {}", target_sn);
        return;
    }

    target_sn_ = target_sn;
    target_format_ = string2PixelFormat(config["format"].as<std::string>("bgr"));
    #ifdef USE_TRT
    use_cuda_cvt_ = config["use_cuda_cvt"].as<bool>(false);
    #endif

    set_PixelFormat(config["pixel_format"].as<std::string>("BayerRG8"));
    set_AcquisitionFrameRateEnable(config["acquisition_frame_rate_enable"].as<bool>(false));
    set_AcquisitionFrameRate(config["acquisition_frame_rate"].as<double>(30.0));
    set_ExposureTime(config["exposure_time"].as<double>(2000.0));
    set_Gain(config["gain"].as<double>(0.0));
    set_Gamma(config["gamma"].as<double>(1.0));
    set_Width(config["width"].as<int>(0));
    set_Height(config["height"].as<int>(0));
    set_OffsetX(config["offset_x"].as<int>(0));
    set_OffsetY(config["offset_y"].as<int>(0));
    set_ReverseX(config["reverse_x"].as<bool>(false));
    set_ReverseY(config["reverse_y"].as<bool>(false));
    set_TriggerMode(config["trigger_mode"].as<std::string>("Off"));
    set_TriggerSource(config["trigger_source"].as<std::string>(""));
    set_TriggerActivation(config["trigger_activation"].as<std::string>(""));
    AWAKENING_INFO("Daheng camera parameters set successfully!");
}

void DahengCamera::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    stop_source_thread();
    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
    if (device_handle_) {
        GXStreamOff(device_handle_);
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
        GXCloseLib();
    }

    AWAKENING_INFO("daheng_camera has stop");
}

bool DahengCamera::start_capture() {
    auto status = GXStreamOn(device_handle_);
    if (status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: GXStreamOn failed: {}", int(status));
        return false;
    }
    running_ = true;
    return true;
}

ImageFrame DahengCamera::read() {
    ImageFrame img_frame;
    if (!device_handle_) {
        AWAKENING_ERROR("daheng_camera: camera is not initialized");
        return img_frame;
    }

    Frame frame;
    frame.frame_data.pImgBuf = frame_buffer_.data();
    frame.frame_data.nImgSize = static_cast<uint32_t>(frame_buffer_.size());
    auto status = GXGetImage(device_handle_, &frame.frame_data, 100);
    if (status != GX_STATUS_SUCCESS || frame.frame_data.nStatus != 0) {
        AWAKENING_ERROR(
            "daheng_camera: GXGetImage failed: status={} frame_status={}",
            int(status),
            int(frame.frame_data.nStatus)
        );
        return img_frame;
    }

    const auto current_time = Clock::now();
    const auto half_exposure = std::chrono::microseconds(static_cast<long>(get_ExposureTime() / 2));
    frame.timestamp = current_time - half_exposure;
    return to_image_frame(frame);
}

void DahengCamera::restart() {
    AWAKENING_WARN("Restarting Daheng camera");
    if (device_handle_) {
        GXStreamOff(device_handle_);
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
    }
    GXCloseLib();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    load(config_);
    auto status = GXStreamOn(device_handle_);
    if (status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: GXStreamOn failed after restart: {}", int(status));
        return;
    }
    AWAKENING_INFO("Daheng camera restarted successfully!");
}

bool DahengCamera::initialize_camera(const std::string& target_sn) {
    auto init_status = GXInitLib();
    if (init_status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: GXInitLib failed: {}", int(init_status));
        return false;
    }

    while (running_) {
        uint32_t device_num = 0;
        auto update_status = GXUpdateDeviceList(&device_num, 1000);
        if (update_status != GX_STATUS_SUCCESS) {
            AWAKENING_ERROR("daheng_camera: GXUpdateDeviceList failed: {}", int(update_status));
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        if (device_num == 0) {
            AWAKENING_ERROR("daheng_camera: no device found");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        GX_OPEN_PARAM open_param {};
        open_param.accessMode = GX_ACCESS_EXCLUSIVE;
        if (target_sn.empty()) {
            open_param.openMode = GX_OPEN_INDEX;
            open_param.pszContent = const_cast<char*>("1");
        } else {
            open_param.openMode = GX_OPEN_SN;
            open_param.pszContent = const_cast<char*>(target_sn.c_str());
        }

        auto open_status = GXOpenDevice(&open_param, &device_handle_);
        if (open_status != GX_STATUS_SUCCESS) {
            AWAKENING_ERROR("daheng_camera: GXOpenDevice failed: {}", int(open_status));
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        int64_t payload_size = 0;
        auto payload_status = GXGetInt(device_handle_, GX_INT_PAYLOAD_SIZE, &payload_size);
        if (payload_status != GX_STATUS_SUCCESS || payload_size <= 0) {
            AWAKENING_ERROR(
                "daheng_camera: GXGetInt(PAYLOAD_SIZE) failed: {}",
                int(payload_status)
            );
            GXCloseDevice(device_handle_);
            device_handle_ = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        frame_buffer_.resize(static_cast<size_t>(payload_size));
        AWAKENING_INFO("Daheng camera initialized successfully");
        return true;
    }
    GXCloseLib();
    return false;
}

double DahengCamera::get_ExposureTime() const {
    double exposure_time = 0.0;
    if (device_handle_) {
        GXGetFloat(device_handle_, GX_FLOAT_EXPOSURE_TIME, &exposure_time);
    }
    return exposure_time;
}

void DahengCamera::set_ExposureTime(double exposure_time) {
    set_float_feature(GX_FLOAT_EXPOSURE_TIME, exposure_time);
}

void DahengCamera::set_Gain(double gain) {
    set_float_feature(GX_FLOAT_GAIN, gain);
}

void DahengCamera::set_Gamma(double gamma) {
    set_bool_feature(GX_BOOL_GAMMA_ENABLE, true);
    set_enum_feature(GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_USER);
    set_float_feature(GX_FLOAT_GAMMA_PARAM, gamma);
}

void DahengCamera::set_Width(int width) {
    if (width > 0) {
        set_int_feature(GX_INT_WIDTH, width);
    }
}

void DahengCamera::set_Height(int height) {
    if (height > 0) {
        set_int_feature(GX_INT_HEIGHT, height);
    }
}

void DahengCamera::set_OffsetX(int offset_x) {
    set_int_feature(GX_INT_OFFSET_X, offset_x);
}

void DahengCamera::set_OffsetY(int offset_y) {
    set_int_feature(GX_INT_OFFSET_Y, offset_y);
}

void DahengCamera::set_ReverseX(bool reverse_x) {
    set_bool_feature(GX_BOOL_REVERSE_X, reverse_x);
}

void DahengCamera::set_ReverseY(bool reverse_y) {
    set_bool_feature(GX_BOOL_REVERSE_Y, reverse_y);
}

void DahengCamera::set_AcquisitionFrameRateEnable(bool enable) {
    set_enum_feature(
        GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
        enable ? GX_ACQUISITION_FRAME_RATE_MODE_ON : GX_ACQUISITION_FRAME_RATE_MODE_OFF
    );
}

void DahengCamera::set_AcquisitionFrameRate(double fps) {
    set_float_feature(GX_FLOAT_ACQUISITION_FRAME_RATE, fps);
}

void DahengCamera::set_TriggerMode(const std::string& mode) {
    set_enum_feature(GX_ENUM_TRIGGER_MODE, trigger_mode_from_string(mode));
}

void DahengCamera::set_TriggerSource(const std::string& source) {
    if (!source.empty()) {
        set_enum_feature(GX_ENUM_TRIGGER_SOURCE, trigger_source_from_string(source));
    }
}

void DahengCamera::set_TriggerActivation(const std::string& activation) {
    if (!activation.empty()) {
        set_enum_feature(GX_ENUM_TRIGGER_ACTIVATION, trigger_activation_from_string(activation));
    }
}

void DahengCamera::set_PixelFormat(const std::string& pixel_format) {
    set_enum_feature(GX_ENUM_PIXEL_FORMAT, pixel_format_from_string(pixel_format));
}

ImageFrame DahengCamera::to_image_frame(Frame& frame) {
    ImageFrame img_frame {
        .format = target_format_,
        .timestamp = frame.timestamp,
    };

    const int width = static_cast<int>(frame.frame_data.nWidth);
    const int height = static_cast<int>(frame.frame_data.nHeight);
    const auto pixel_format = frame.frame_data.nPixelFormat;
    if (pixel_format == pixel_format_from_string("Mono8")) {
        cv::Mat mono(cv::Size(width, height), CV_8UC1, frame.frame_data.pImgBuf);
        if (target_format_ == PixelFormat::GRAY) {
            img_frame.src_img = mono.clone();
        } else {
            cv::cvtColor(mono, img_frame.src_img, cv::COLOR_GRAY2BGR);
        }
        return img_frame;
    }

    cv::Mat raw(cv::Size(width, height), CV_8UC1, frame.frame_data.pImgBuf);
    int cvt_code =
        (target_format_ == PixelFormat::RGB) ? cv::COLOR_BayerRG2BGR : cv::COLOR_BayerRG2RGB;
    #ifdef USE_TRT
    if (use_cuda_cvt_) {
        static utils::__cuda::CvtColor cvt;
        cvt.process(raw, img_frame.src_img, cvt_code);
    } else {
    #endif
        cv::cvtColor(raw, img_frame.src_img, cvt_code);
    #ifdef USE_TRT
    }
    #endif
    return img_frame;
}

void DahengCamera::set_enum_feature(GX_FEATURE_ID_CMD feature, int64_t value) {
    auto status = GXSetEnum(device_handle_, feature, value);
    if (status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: set enum {} failed: {}", int(feature), int(status));
    }
}

void DahengCamera::set_bool_feature(GX_FEATURE_ID_CMD feature, bool value) {
    auto status = GXSetBool(device_handle_, feature, value);
    if (status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: set bool {} failed: {}", int(feature), int(status));
    }
}

void DahengCamera::set_float_feature(GX_FEATURE_ID_CMD feature, double value) {
    GX_FLOAT_RANGE range {};
    auto range_status = GXGetFloatRange(device_handle_, feature, &range);
    if (range_status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR(
            "daheng_camera: get float range {} failed: {}",
            int(feature),
            int(range_status)
        );
        return;
    }
    value = std::clamp(value, range.dMin, range.dMax);
    auto status = GXSetFloat(device_handle_, feature, value);
    if (status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: set float {} failed: {}", int(feature), int(status));
    }
}

void DahengCamera::set_int_feature(GX_FEATURE_ID_CMD feature, int64_t value) {
    GX_INT_RANGE range {};
    auto range_status = GXGetIntRange(device_handle_, feature, &range);
    if (range_status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR(
            "daheng_camera: get int range {} failed: {}",
            int(feature),
            int(range_status)
        );
        return;
    }
    value = std::clamp(value, range.nMin, range.nMax);
    auto status = GXSetInt(device_handle_, feature, value);
    if (status != GX_STATUS_SUCCESS) {
        AWAKENING_ERROR("daheng_camera: set int {} failed: {}", int(feature), int(status));
    }
}

int64_t DahengCamera::pixel_format_from_string(const std::string& pixel_format) const {
    const auto fmt = utils::to_upper(pixel_format);
    if (fmt == "MONO8") {
        return GX_PIXEL_FORMAT_MONO8;
    }
    if (fmt == "RGB8PACKED" || fmt == "RGB8") {
        return GX_PIXEL_FORMAT_RGB8_PLANAR;
    }
    return GX_PIXEL_FORMAT_BAYER_RG8;
}

int64_t DahengCamera::trigger_mode_from_string(const std::string& mode) const {
    return (utils::to_upper(mode) == "ON") ? GX_TRIGGER_MODE_ON : GX_TRIGGER_MODE_OFF;
}

int64_t DahengCamera::trigger_source_from_string(const std::string& source) const {
    const auto val = utils::to_upper(source);
    if (val == "SOFTWARE") {
        return GX_TRIGGER_SOURCE_SOFTWARE;
    }
    if (val == "LINE2") {
        return GX_TRIGGER_SOURCE_LINE2;
    }
    return GX_TRIGGER_SOURCE_LINE0;
}

int64_t DahengCamera::trigger_activation_from_string(const std::string& activation) const {
    const auto val = utils::to_upper(activation);
    if (val == "FALLINGEDGE") {
        return GX_TRIGGER_ACTIVATION_FALLINGEDGE;
    }
    return GX_TRIGGER_ACTIVATION_RISINGEDGE;
}

} // namespace awakening
#endif
