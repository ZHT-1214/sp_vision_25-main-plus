#pragma once

#include <filesystem>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace awakening {

class VideoSaver {
public:
    enum class Mode { Blocking, NonBlocking };
    static inline std::string generate_record_filename(const std::string& folder_path) {
        namespace fs = std::filesystem;

        fs::create_directories(folder_path);

        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);

        std::tm tm {};

#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".avi";

        return (fs::path(folder_path) / oss.str()).string();
    }
    explicit VideoSaver(const std::string& filename, Mode mode = Mode::NonBlocking):
        save_path_(filename),
        mode_(mode) {
        if (mode_ == Mode::NonBlocking) {
            running_ = true;
            worker_ = std::thread(&VideoSaver::worker_loop, this);
        }
    }

    ~VideoSaver() {
        close();
    }

    bool write_frame(
        const cv::Mat& frame,
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        double fps = 30.0
    ) {
        if (frame.empty())
            return false;

        auto now = Clock::now();

        const auto min_interval = std::chrono::duration<double>(1.0 / fps);

        if (last_write_time_.time_since_epoch().count() != 0
            && now - last_write_time_ < min_interval) {
            return false; // 丢帧
        }

        last_write_time_ = now;

        if (mode_ == Mode::Blocking)
            return write_impl(frame, codec, fps);

        {
            std::lock_guard lock(mutex_);
            queue_.emplace(FrameTask { frame.clone(), codec, fps });
        }

        cv_.notify_one();
        return true;
    }
    bool opened() const {
        return is_opened_;
    }

    void close() {
        if (closed_.exchange(true))
            return;

        if (mode_ == Mode::NonBlocking) {
            running_ = false;
            cv_.notify_all();

            if (worker_.joinable())
                worker_.join();
        }

        if (is_opened_) {
            writer_.release();
            is_opened_ = false;
        }
    }

private:
    using Clock = std::chrono::steady_clock;

    struct FrameTask {
        cv::Mat frame;
        int codec;
        double fps;
    };

    bool write_impl(const cv::Mat& frame, int codec, double fps) {
        if (!is_opened_) {
            writer_.open(save_path_, codec, fps, frame.size(), frame.channels() == 3);

            is_opened_ = writer_.isOpened();

            if (!is_opened_)
                return false;
        }

        writer_.write(frame);
        return true;
    }

    void worker_loop() {
        while (running_ || !queue_.empty()) {
            FrameTask task;

            {
                std::unique_lock lock(mutex_);

                cv_.wait(lock, [&] { return !running_ || !queue_.empty(); });

                if (queue_.empty())
                    continue;

                task = std::move(queue_.front());
                queue_.pop();
            }

            write_impl(task.frame, task.codec, task.fps);
        }
    }

private:
    std::string save_path_;

    cv::VideoWriter writer_;
    std::atomic<bool> is_opened_ { false };

    Mode mode_;

    Clock::time_point last_write_time_ {};

    // async
    std::atomic<bool> running_ { false };
    std::atomic<bool> closed_ { false };

    std::thread worker_;

    std::mutex mutex_;
    std::condition_variable cv_;

    std::queue<FrameTask> queue_;
};

} // namespace awakening