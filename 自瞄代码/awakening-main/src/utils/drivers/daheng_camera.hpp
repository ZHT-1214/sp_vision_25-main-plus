#pragma once

#include "utils/common/image.hpp"
#include "utils/common/type_common.hpp"
#include "utils/drivers/camera.hpp"
#include "utils/logger.hpp"
#include "utils/scheduler/scheduler.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "GxIAPI.h"

namespace awakening {

class DahengCamera: public Camera {
public:
    struct Frame {
        GX_FRAME_DATA frame_data {};
        TimePoint timestamp {};
    };

    DahengCamera(const YAML::Node& config, Scheduler& scheduler):
        Camera(&scheduler),
        scheduler_ref_(scheduler) {
        config_ = config;
        running_ = true;
    }

    ~DahengCamera() {
        stop();
    }

    void init() override;
    void load(const YAML::Node& config);
    void stop() override;
    void restart() override;
    bool is_running() const override {
        return running_;
    }
    ImageFrame read() override;
    bool start_capture() override;
    bool initialize_camera(const std::string& target_sn);
    double get_ExposureTime() const override;
    void set_ExposureTime(double exposure_time) override;
    void set_Gain(double gain);
    void set_Gamma(double gamma);
    void set_Width(int width);
    void set_Height(int height);
    void set_OffsetX(int offset_x);
    void set_OffsetY(int offset_y);
    void set_ReverseX(bool reverse_x);
    void set_ReverseY(bool reverse_y);
    void set_AcquisitionFrameRateEnable(bool enable);
    void set_AcquisitionFrameRate(double fps);
    void set_TriggerMode(const std::string& mode);
    void set_TriggerSource(const std::string& source);
    void set_TriggerActivation(const std::string& activation);
    void set_PixelFormat(const std::string& pixel_format);

    template<typename Tag>
    void start(std::string source_name) {
        using IO = IOPair<Tag, ImageFrame>;
        source_snapshot_id_ = scheduler_ref_.register_source<IO>(source_name);
        auto status = GXStreamOn(device_handle_);
        if (status != GX_STATUS_SUCCESS) {
            AWAKENING_ERROR("daheng_camera: GXStreamOn failed: {}", int(status));
            return;
        }
        running_ = true;
        daemon_thread_ = std::thread(&DahengCamera::run_loop<IO>, this);
    }

    template<typename IO>
    void run_loop() {
        while (running_) {
            daheng_capture_loop<IO>();
            if (!running_) {
                break;
            }
            restart();
        }
    }

    template<typename IO>
    void daheng_capture_loop() {
        AWAKENING_INFO("Starting Daheng image capture loop!");
        int fail_count = 0;
        while (running_) {
            Frame frame;
            frame.frame_data.pImgBuf = frame_buffer_.data();
            frame.frame_data.nImgSize = static_cast<uint32_t>(frame_buffer_.size());
            auto status = GXGetImage(device_handle_, &frame.frame_data, 100);
            if (status == GX_STATUS_SUCCESS && frame.frame_data.nStatus == 0) {
                const auto current_time = Clock::now();
                const auto half_exposure =
                    std::chrono::microseconds(static_cast<long>(get_ExposureTime() / 2));
                frame.timestamp = current_time - half_exposure;
                auto img_frame = to_image_frame(frame);
                scheduler_ref_.runtime_push_source<IO>(
                    source_snapshot_id_,
                    [f = std::move(img_frame)]() mutable {
                        return std::make_tuple(std::optional<typename IO::second_type>(std::move(f))
                        );
                    }
                );
                fail_count = 0;
            } else {
                AWAKENING_ERROR(
                    "daheng_camera: GXGetImage failed: status={} frame_status={}",
                    int(status),
                    int(frame.frame_data.nStatus)
                );
                fail_count++;
                if (fail_count > 10) {
                    break;
                }
            }
        }
        AWAKENING_INFO("Exiting Daheng image capture loop.");
    }

    ImageFrame to_image_frame(Frame& frame);

    std::atomic<bool> running_ { false };

private:
    void set_enum_feature(GX_FEATURE_ID_CMD feature, int64_t value);
    void set_bool_feature(GX_FEATURE_ID_CMD feature, bool value);
    void set_float_feature(GX_FEATURE_ID_CMD feature, double value);
    void set_int_feature(GX_FEATURE_ID_CMD feature, int64_t value);
    int64_t pixel_format_from_string(const std::string& pixel_format) const;
    int64_t trigger_mode_from_string(const std::string& mode) const;
    int64_t trigger_source_from_string(const std::string& source) const;
    int64_t trigger_activation_from_string(const std::string& activation) const;

    YAML::Node config_;
    Scheduler& scheduler_ref_;
    std::thread daemon_thread_;
    std::string target_sn_;
    PixelFormat target_format_ = PixelFormat::BGR;
    bool use_cuda_cvt_ = false;

    GX_DEV_HANDLE device_handle_ = nullptr;
    std::vector<uint8_t> frame_buffer_;
};

} // namespace awakening
