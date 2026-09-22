#include "_rerun/recorder.hpp"
#include "param_deliver.h"
#include "tasks/auto_aim/armor_track/motion_model.hpp"
#include "tasks/base/web.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rerun.hpp>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace awakening::rerun_visual {
namespace {
    std::string env_or(const char* name, const char* fallback) {
        if (const char* value = std::getenv(name)) {
            return value;
        }
        return fallback;
    }

    int env_int(const char* name, int fallback, int minimum, int maximum) {
        try {
            return std::clamp(
                std::stoi(env_or(name, std::to_string(fallback).c_str())),
                minimum,
                maximum
            );
        } catch (...) {
            return fallback;
        }
    }

    std::string entity_part(std::string value) {
        std::replace_if(
            value.begin(),
            value.end(),
            [](char c) { return c == '/' || c == ' ' || c == '.'; },
            '_'
        );
        return value;
    }

    rerun::datatypes::Vec3D vec3(const Eigen::Vector3d& v) {
        return { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
    }

    rerun::datatypes::Quaternion quat(const Eigen::Matrix3d& rotation) {
        const Eigen::Quaterniond q(rotation);
        return rerun::datatypes::Quaternion::from_xyzw(
            static_cast<float>(q.x()),
            static_cast<float>(q.y()),
            static_cast<float>(q.z()),
            static_cast<float>(q.w())
        );
    }

    std::string frame_name(int frame_id) {
        static constexpr std::array<std::string_view, 11> names = {
            "odom",    "gimbal_odom", "gimbal",    "camera", "camera_cv", "shoot",
            "big_yaw", "omni_0",      "omni_0_cv", "omni_1", "omni_1_cv"
        };
        if (frame_id >= 0 && static_cast<size_t>(frame_id) < names.size()) {
            return std::string(names[frame_id]);
        }
        return "frame_" + std::to_string(frame_id);
    }

    float armor_width(auto_aim::ArmorClass number) {
        using namespace auto_aim;
        switch (armor_type_by_armor_class(number)) {
            case ArmorType::SimpleSmall:
                return ArmorTypeTraits<ArmorType::SimpleSmall>::WIDTH;
            case ArmorType::Large:
                return ArmorTypeTraits<ArmorType::Large>::WIDTH;
        }
        return ArmorTypeTraits<ArmorType::SimpleSmall>::WIDTH;
    }
} // namespace

struct Recorder::Impl {
    rerun::RecordingStream stream { "awakening" };
    bool active = false;
    std::mutex mutex;
    std::unordered_set<std::string> initialized_frames;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_json;
    std::chrono::steady_clock::time_point last_vision {};
    std::mutex image_mutex;
    std::condition_variable image_ready;
    cv::Mat latest_image;
    std::string latest_image_entity;
    bool image_pending = false;
    bool stop_image_worker = false;
    std::thread image_worker;

    Impl() {
        const auto mode = env_or("AWAKENING_RERUN", "off");
        if (mode == "0" || mode == "off" || mode == "false") {
            return;
        }

        rerun::Error error;
        if (mode == "connect") {
            error = stream.connect_grpc(
                env_or("AWAKENING_RERUN_URL", "rerun+http://127.0.0.1:9876/proxy")
            );
        } else if (mode == "save") {
            error = stream.save(env_or("AWAKENING_RERUN_PATH", "awakening.rrd"));
        } else if (mode == "serve") {
            auto result = stream.serve_grpc(
                env_or("AWAKENING_RERUN_BIND", "0.0.0.0"),
                static_cast<uint16_t>(std::stoi(env_or("AWAKENING_RERUN_PORT", "9876")))
            );
            if (result.is_err()) {
                error = result.error;
            }
        } else {
            rerun::SpawnOptions options;
            options.server_memory_limit = "8MB";
            options.memory_limit = "25%";
            const std::string viewer_path = std::string(ROOT_DIR) + "/script/rerun_latest.sh";
            options.executable_path = viewer_path;
            error = stream.spawn(options);
        }
        active = error.is_ok();
        if (active) {
            stream.log_static(
                "world",
                rerun::CoordinateFrame("odom"),
                rerun::ViewCoordinates::RIGHT_HAND_Z_UP
            );
            stream.log_static(
                "status/rerun",
                rerun::TextDocument("Rerun visualization initialized")
            );
            image_worker = std::thread([this] { run_image_worker(); });
        }
    }

    ~Impl() {
        {
            std::lock_guard lock(image_mutex);
            stop_image_worker = true;
            image_pending = false;
        }
        image_ready.notify_one();
        if (image_worker.joinable()) {
            image_worker.join();
        }
    }

    void run_image_worker() {
        auto next_send = std::chrono::steady_clock::now();
        while (true) {
            cv::Mat image;
            std::string entity;
            {
                std::unique_lock lock(image_mutex);
                image_ready.wait(lock, [this] { return image_pending || stop_image_worker; });
                if (stop_image_worker) {
                    return;
                }
                image_ready.wait_until(lock, next_send, [this] { return stop_image_worker; });
                if (stop_image_worker) {
                    return;
                }
                image = std::move(latest_image);
                entity = std::move(latest_image_entity);
                image_pending = false;
            }

            const int max_width = env_int("AWAKENING_RERUN_IMAGE_WIDTH", 640, 320, 3840);
            cv::Mat encoded_input = image;
            cv::Mat resized;
            if (image.cols > max_width) {
                const double scale = static_cast<double>(max_width) / image.cols;
                cv::resize(image, resized, {}, scale, scale, cv::INTER_AREA);
                encoded_input = resized;
            }

            std::vector<uint8_t> jpeg;
            const int quality = env_int("AWAKENING_RERUN_JPEG_QUALITY", 30, 10, 95);
            const std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, quality };
            if (cv::imencode(".jpg", encoded_input, jpeg, params)) {
                stream.log(
                    entity,
                    rerun::EncodedImage::from_bytes(
                        rerun::take_ownership(std::move(jpeg)),
                        rerun::MediaType::jpeg()
                    )
                );
            }
            const int max_fps = env_int("AWAKENING_RERUN_IMAGE_FPS", 8, 1, 30);
            next_send =
                std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / max_fps);
            // If producers submitted several frames while encoding/sending, only the newest
            // remains in latest_image and is processed by the next iteration.
        }
    }

    void flatten_json(const std::string& path, const nlohmann::json& value) {
        if (value.is_number()) {
            stream.log(path, rerun::Scalars(value.get<double>()));
        } else if (value.is_boolean()) {
            stream.log(path, rerun::Scalars(value.get<bool>() ? 1.0 : 0.0));
        } else if (value.is_object()) {
            for (auto it = value.begin(); it != value.end(); ++it) {
                // Keep every scalar directly below one origin so Rerun creates one plot view.
                flatten_json(path + "_" + entity_part(it.key()), it.value());
            }
        } else if (value.is_array() && !value.empty() && value.back().is_number()) {
            // Web keeps rolling arrays; Rerun owns the history, so only append the newest sample.
            stream.log(path, rerun::Scalars(value.back().get<double>()));
        } else if (value.is_string()) {
            // A document shows the current value. TextLog would append rows and auto-scroll.
            stream.log(path, rerun::TextDocument(value.get<std::string>()));
        }
    }
};

Recorder& Recorder::instance() {
    static Recorder recorder;
    return recorder;
}

Recorder::Recorder(): impl_(std::make_unique<Impl>()) {}
Recorder::~Recorder() = default;

bool Recorder::enabled() const noexcept {
    return impl_->active;
}

void Recorder::log_image(const cv::Mat& input, const std::string& entity) {
    if (!enabled() || input.empty()) {
        return;
    }
    {
        std::lock_guard lock(impl_->image_mutex);
        input.copyTo(impl_->latest_image); // Single slot: overwrites any unsent frame.
        impl_->latest_image_entity = entity;
        impl_->image_pending = true;
    }
    impl_->image_ready.notify_one();
}

void Recorder::log_json(const std::string& root, const nlohmann::json& value) {
    if (!enabled()) {
        return;
    }
    std::lock_guard lock(impl_->mutex);
    const auto now = std::chrono::steady_clock::now();
    auto& last = impl_->last_json[root];
    if (now - last < std::chrono::milliseconds(100)) {
        return;
    }
    last = now;
    impl_->flatten_json("data/logs/" + entity_part(root), value);
}

void Recorder::log_transform(
    const std::string& parent,
    const std::string& child,
    const ISO3& child_in_parent
) {
    if (!enabled()) {
        return;
    }
    std::lock_guard lock(impl_->mutex);
    for (const auto& frame: { parent, child }) {
        if (impl_->initialized_frames.insert(frame).second) {
            impl_->stream.log_static(
                "frames/" + entity_part(frame),
                rerun::CoordinateFrame(frame),
                rerun::TransformAxes3D(0.2F)
            );
        }
    }
    impl_->stream.log(
        "transforms/" + entity_part(parent) + "_to_" + entity_part(child),
        rerun::Transform3D(vec3(child_in_parent.translation()), quat(child_in_parent.rotation()))
            .with_parent_frame(parent)
            .with_child_frame(child)
    );
}

void Recorder::log_vision(const VisionDebugCtx& ctx) {
    if (!enabled()) {
        return;
    }
    std::lock_guard lock(impl_->mutex);
    const auto now = std::chrono::steady_clock::now();
    if (now - impl_->last_vision < std::chrono::milliseconds(66)) {
        return;
    }
    impl_->last_vision = now;
    const auto target = ctx.armor_target.get();
    if (target.check()) {
        auto state = target.get_target_state();
        state.predict(Clock::now(), target.target_number);
        const auto armor_poses = state.get_armors_pose(target.target_number);
        std::vector<rerun::datatypes::Vec3D> centers;
        std::vector<rerun::datatypes::Vec3D> sizes;
        std::vector<rerun::datatypes::Quaternion> rotations;
        std::vector<std::string> labels;
        centers.reserve(armor_poses.size());
        sizes.reserve(armor_poses.size());
        rotations.reserve(armor_poses.size());
        labels.reserve(armor_poses.size());
        for (size_t i = 0; i < armor_poses.size(); ++i) {
            const auto& pose = armor_poses[i];
            centers.push_back(vec3(pose.translation()));
            sizes.emplace_back(0.03F, armor_width(target.target_number), 0.135F);
            rotations.push_back(quat(pose.rotation()));
            labels.push_back(
                auto_aim::string_by_armor_class(target.target_number) + "_" + std::to_string(i)
            );
        }
        impl_->stream.log(
            "frames/target/armors",
            rerun::Boxes3D::from_centers_and_sizes(centers, sizes)
                .with_quaternions(rotations)
                .with_labels(labels)
                .with_colors({ rerun::Color(40, 120, 255) }),
            rerun::CoordinateFrame(frame_name(state.frame_id))
        );
        impl_->stream.log(
            "frames/target/center",
            rerun::Points3D({ vec3(state.pos()) })
                .with_radii({ 0.06F })
                .with_colors({ rerun::Color(0, 255, 0) }),
            rerun::CoordinateFrame(frame_name(state.frame_id))
        );
        impl_->stream.log(
            "frames/target/velocity",
            rerun::Arrows3D::from_vectors({ vec3(state.vel()) })
                .with_origins({ vec3(state.pos()) })
                .with_colors({ rerun::Color(255, 255, 0) }),
            rerun::CoordinateFrame(frame_name(state.frame_id))
        );
    } else {
        impl_->stream.log("frames/target/armors", rerun::Clear::FLAT);
        impl_->stream.log("frames/target/center", rerun::Clear::FLAT);
        impl_->stream.log("frames/target/velocity", rerun::Clear::FLAT);
    }
}

} // namespace awakening::rerun_visual
