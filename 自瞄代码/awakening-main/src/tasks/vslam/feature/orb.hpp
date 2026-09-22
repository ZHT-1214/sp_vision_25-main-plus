#pragma once
#include "tasks/vslam/frame.hpp"
#include "tasks/vslam/type.hpp"
#include <algorithm>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>
#include <yaml-cpp/node/node.h>
namespace awakening::vslam {
class Orb {
public:
    struct Params {
        int nfeatures;
        float scale_factor;
        int nlevels;
        int edge_threshold;
        int fast_threshold;
        double match_threshold;
        void load(const YAML::Node& config) {
            nfeatures = config["nfeatures"].as<int>();
            scale_factor = config["scale_factor"].as<float>();
            nlevels = config["nlevels"].as<int>();
            edge_threshold = config["edge_threshold"].as<int>();
            fast_threshold = config["fast_threshold"].as<int>();
            match_threshold = config["match_threshold"].as<double>();
        }
    } params_;
    Orb(const YAML::Node& config) {
        params_.load(config);
        detector_ = cv::ORB::create(
            params_.nfeatures,
            params_.scale_factor,
            params_.nlevels,
            params_.edge_threshold,
            0,
            2,
            cv::ORB::HARRIS_SCORE,
            31,
            params_.fast_threshold
        );
        extractor_ = detector_;
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    }
    void detect(Frame& frame) {
        detector_->detect(frame.img_gray, frame.keypoints);
        extractor_->compute(
            frame.img_gray,
            frame.keypoints,
            frame.descriptors
        );
        frame.detected = true;
    }
    // Match match(const Feature& feature1, const Feature& feature2) {
    //     Match match;
    //     match.features = std::make_pair(feature1, feature2);
    //     match.timestamp = feature2.timestamp;
    //     match.frame_id = feature2.frame_id;
    //     if (!feature1.detected || !feature2.detected) {
    //         return match;
    //     }
    //     const auto& desc1 = feature1.detected.value().descriptors;
    //     const auto& desc2 = feature2.detected.value().descriptors;
    //     if (desc1.empty() || desc2.empty()) {
    //         return match;
    //     }
    //     std::vector<cv::DMatch> raw_matches;
    //     matcher_->match(desc1, desc2, raw_matches);
    //     if (raw_matches.empty()) {
    //         return match;
    //     }
    //     auto min_max = std::minmax_element(
    //         raw_matches.begin(),
    //         raw_matches.end(),
    //         [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; }
    //     );
    //     double min_distance = min_max.first->distance;
    //     double distance_threshold = params_.match_threshold;
    //     match.matches.reserve(raw_matches.size());
    //     for (const auto& raw_match: raw_matches) {
    //         if (raw_match.distance <= distance_threshold) {
    //             match.matches.push_back(raw_match);
    //         }
    //     }
    //     return match;
    // }

    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
};
} // namespace awakening::vslam
