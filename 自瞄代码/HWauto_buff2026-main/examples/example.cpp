#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "buff_algo/detector/buff_detector.hpp"
#include "buff_algo/locate/buff_pnp_solver.hpp"
#include "buff_algo/predictor/buff_predictor.hpp"
#include "buff_algo/selector/buff_selector.hpp"
#include "buff_algo/tracker/buff_tracker.hpp"
#include "buff_algo/tracker/tracker_status.hpp"

namespace {

// 示例默认相机内参（1280x768 画面），实车请替换为标定结果。
const cv::Mat kCameraMatrix = (cv::Mat_<double>(3, 3) << 800.0, 0.0, 640.0,
                                                               0.0, 800.0, 384.0,
                                                               0.0, 0.0, 1.0);
const cv::Mat kDistortion = cv::Mat::zeros(1, 5, CV_64F);

const std::array<cv::Scalar, 9> kKeypointColors = {
    cv::Scalar(0, 255, 0),    // 0
    cv::Scalar(255, 0, 0),    // 1
    cv::Scalar(0, 0, 255),    // 2
    cv::Scalar(255, 255, 0),  // 3
    cv::Scalar(255, 0, 255),  // 4
    cv::Scalar(0, 255, 255),  // 5
    cv::Scalar(128, 0, 255),  // 6
    cv::Scalar(255, 128, 0),  // 7
    cv::Scalar(255, 255, 255) // 8  R中心
};

// 只画模型输出的 9 个关键点（编号 0..8）。
void draw_keypoints(cv::Mat& image, const buff_algo::DetectedBuff& detected) {
    for (std::size_t index = 0; index < detected.keypoints.size(); ++index) {
        const cv::Point point(
            cvRound(detected.keypoints[index].x()),
            cvRound(detected.keypoints[index].y())
        );
        cv::circle(image, point, 4, kKeypointColors[index], cv::FILLED, cv::LINE_AA);
        cv::putText(
            image,
            std::to_string(index),
            point + cv::Point(6, -6),
            cv::FONT_HERSHEY_SIMPLEX,
            0.4,
            kKeypointColors[index],
            1,
            cv::LINE_AA
        );
    }
}

using Detector = buff_algo::BuffDetector;

std::vector<buff_algo::DetectedBuff> run_detector(
    Detector& detector,
    const cv::Mat& frame
) {
    return detector.detect_with_keypoints(frame);
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc > 5) {
        std::cerr
            << "Usage: " << argv[0]
            << " [video=buff.mp4] [output=buff_detected.mp4] [engine=config/buff_model_name] [confidence=config]\n"
            << "engine / confidence 省略时取 config/detector/detector.yaml 中的参数\n";
        return 1;
    }

    try {
        cv::FileStorage detector_config(
            BUFF_ALGO_CONFIG_DIR "/detector/detector.yaml",
            cv::FileStorage::READ
        );
        cv::FileStorage tracker_config(
            BUFF_ALGO_CONFIG_DIR "/tracker/tracker.yaml",
            cv::FileStorage::READ
        );
        cv::FileStorage predictor_config(
            BUFF_ALGO_CONFIG_DIR "/predictor/predictor.yaml",
            cv::FileStorage::READ
        );
        if (!detector_config.isOpened() || !tracker_config.isOpened() ||
            !predictor_config.isOpened()) {
            throw std::runtime_error("failed to open config yaml in " BUFF_ALGO_CONFIG_DIR);
        }

        const cv::FileNode detector_node = detector_config["buff_detector"];
        const std::string default_model = static_cast<std::string>(detector_node["buff_model_name"]);
        const float default_confidence =
            static_cast<float>(detector_node["buff_confidence_threshold"]);
        const float nms_threshold = static_cast<float>(detector_node["buff_nms_threshold"]);
        const int mode = static_cast<int>(detector_node["buff_mode"]);
        const int buff_color = static_cast<int>(detector_node["buff_color"]);

        const std::string video_path = argc >= 2 ? argv[1] : "buff.mp4";
        const std::string output_path = argc >= 3 ? argv[2] : "buff_detected.mp4";
        const std::string engine_path = argc >= 4 ? argv[3] : default_model;
        const float confidence = argc >= 5 ? std::stof(argv[4]) : default_confidence;

        const buff_algo::BuffDetectorConfig config = {
            .confidence_threshold = confidence,
            .nms_threshold = nms_threshold
        };

        buff_algo::BuffDetector detector(engine_path, config);

        // 完整链路：检测 -> 筛选 -> PnP -> 跟踪 -> 预测
        buff_algo::BuffSelector selector(buff_color);
        selector.set_camera_matrix(kCameraMatrix, kDistortion);

        buff_algo::BuffPnPSolver solver;
        solver.set_camera_matrix(kCameraMatrix, kDistortion);

        buff_algo::BuffTracker tracker(tracker_config["buff_tracker"]);
        tracker.set_mode(static_cast<buff_algo::BuffMode>(mode));

        buff_algo::BuffPredictor predictor(predictor_config["buff_predictor"]);

        cv::VideoCapture capture(video_path);
        if (!capture.isOpened()) {
            throw std::runtime_error("failed to open video: " + video_path);
        }

        const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
        const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
        double fps = capture.get(cv::CAP_PROP_FPS);
        if (!std::isfinite(fps) || fps <= 0.0) {
            fps = 30.0;
        }

        cv::VideoWriter writer(
            output_path,
            cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
            fps,
            {width, height}
        );
        if (!writer.isOpened()) {
            throw std::runtime_error("failed to create output video: " + output_path);
        }

        cv::Mat frame;
        int processed = 0;
        bool window_ok = true;
        buff_algo::StatusType last_status = buff_algo::StatusType::LOST;
        while (capture.read(frame)) {
            const std::vector<buff_algo::DetectedBuff> detected = run_detector(detector, frame);

            std::vector<buff_algo::BuffDetection> detections;
            detections.reserve(detected.size());
            for (const auto& item : detected) {
                detections.push_back(item.detection);
            }

            std::vector<buff_algo::BuffDetection> selected;
            selector.select_buffs(detections, selected);

            const std::vector<buff_algo::Pose3f> poses = solver.solve_pnp(selected);
            if (!poses.empty()) {
                tracker.push(buff_algo::Buff(poses[0]));
            }

            const double timestamp = capture.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
            tracker.update(timestamp);

            buff_algo::BuffState state = tracker.get_state();
            predictor.set_state(state, timestamp, timestamp);
            predictor.predict_position(0.1f);

            // 状态切换或每 30 帧打印一次 tracker 自带的调试信息
            if (tracker.status() != last_status || processed % 30 == 0) {
                tracker.print_colored_status_info();
                last_status = tracker.status();
            }

            for (const auto& item : detected) {
                draw_keypoints(frame, item);
            }

            writer.write(frame);

            if (window_ok) {
                try {
                    cv::imshow("buff detection", frame);
                    // 按原始视频帧率播放，避免 GPU 处理过快导致画面快进
                    const int frame_delay = static_cast<int>(std::round(1000.0 / fps));
                    const int key = cv::waitKey(std::max(1, frame_delay));
                    if (key == 'q' || key == 27) {
                        break;
                    }
                } catch (const cv::Exception&) {
                    window_ok = false;
                }
            }

            ++processed;
        }

        tracker.print_colored_status_info();
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
