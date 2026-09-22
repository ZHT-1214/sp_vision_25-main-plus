#pragma once
#include "utils/common/image.hpp"
#include "utils/drivers/camera.hpp"
#include "utils/logger.hpp"
#include "utils/scheduler/scheduler.hpp"
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <yaml-cpp/node/node.h>
namespace awakening {
class VideoPlayer: public Camera {
public:
    VideoPlayer(const YAML::Node& config, Scheduler& scheduler):
        Camera(&scheduler),
        scheduler_ref_(scheduler) {
        path_ = config["path"].as<std::string>();
        fps_ = config["fps"].as<int>();
        loop_ = config["loop"].as<bool>();
        start_frame_ = config["start_frame"].as<int>();
    }
    ~VideoPlayer() {
        stop();
    }
    template<typename Tag>
    void start(std::string source_name) {
        cap_.open(path_);
        if (!cap_.isOpened()) {
            AWAKENING_ERROR("open {} failed", path_);
        }
        cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
        using IO = IOPair<Tag, ImageFrame>;
        source_snapshot_id_ = scheduler_ref_.register_source<IO>(source_name);
        running_ = true;
        worker_ = std::thread(&VideoPlayer::run_loop<IO>, this);
    }
    bool start_capture() override {
        cap_.open(path_);
        if (!cap_.isOpened()) {
            AWAKENING_ERROR("open {} failed", path_);
            return false;
        }
        cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
        next_frame_time_ = Clock::now();
        running_ = true;
        return true;
    }
    bool is_running() const override {
        return running_;
    }
    ImageFrame read() override {
        const auto frame_interval =
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / fps_));
        std::this_thread::sleep_until(next_frame_time_);
        next_frame_time_ += frame_interval;

        cv::Mat frame_bgr;
        cap_ >> frame_bgr;

        if (frame_bgr.empty()) {
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
            } else {
                running_ = false;
            }
            return {};
        }

        ImageFrame frame;
        frame.src_img = std::move(frame_bgr);
        frame.timestamp = Clock::now();
        frame.format = PixelFormat::BGR;
        return frame;
    }
    void stop() override {
        running_ = false;
        stop_source_thread();
        if (worker_.joinable()) {
            worker_.join();
        }
        cap_.release();
        AWAKENING_INFO("Video closed successfully: {}", path_);
    }
    template<typename IO>
    void run_loop() {
        const auto frame_interval =
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / fps_));

        auto next_frame_time = Clock::now();

        while (running_) {
            cv::Mat frame_bgr;
            cap_ >> frame_bgr;

            if (frame_bgr.empty()) {
                if (loop_) {
                    cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
                    continue;
                } else {
                    break;
                }
            }

            ImageFrame frame;
            frame.src_img = std::move(frame_bgr);
            frame.timestamp = Clock::now();
            frame.format = PixelFormat::BGR;
            scheduler_ref_.runtime_push_source<IO>(source_snapshot_id_, [f = std::move(frame)]() {
                return std::make_tuple(std::optional<typename IO::second_type>(std::move(f)));
            });

            next_frame_time += frame_interval;
            std::this_thread::sleep_until(next_frame_time);
        }
    }
    std::string path_;
    int fps_;
    bool loop_;
    int start_frame_;
    std::atomic<bool> running_ { false };
    cv::VideoCapture cap_;
    std::thread worker_;
    Scheduler& scheduler_ref_;
    TimePoint next_frame_time_ {};
};
} // namespace awakening
