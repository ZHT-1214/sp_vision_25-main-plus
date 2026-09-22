#pragma once

#include <chrono>
#include <expected>
#include <opencv2/core.hpp>
#include <optional>
#include <thread>

#include "shm_layout.hpp"
#include "shm_region.hpp"
#include "shm_triple_buffer.hpp"

namespace talos::ipc {

/**
 * @brief 共享内存 IPC 客户端
 *
 * 提供高级 API 用于:
 * - 订阅图像流 (Rust → C++)
 * - 订阅位姿数据 (Rust → C++)
 * - 发布云台控制命令 (C++ → Rust)
 *
 * 使用示例:
 * ```cpp
 * auto client = ShmClient::connect();
 * if (!client) {
 *     std::cerr << "连接失败: " << to_string(client.error()) << std::endl;
 *     return;
 * }
 *
 * // 订阅图像
 * if (auto frame = client->recv_image()) {
 *     cv::Mat img = frame->image;
 *     // 处理图像...
 * }
 *
 * // 发布云台命令
 * client->send_gimbal_cmd(15.0f, -8.0f, 3.5f, true);
 * ```
 */
class ShmClient {
public:
    /**
     * @brief 接收到的图像帧
     */
    struct ImageFrame {
        cv::Mat image; // 零拷贝 cv::Mat (直接指向共享内存)
        uint64_t seq; // 帧序号
        uint64_t timestamp_ns; // 时间戳
    };

    /**
     * @brief 接收到的位姿数据
     */
    struct Pose {
        double x, y, z; // 位置 (米)
        double qw, qx, qy, qz; // 四元数
        uint64_t frame_seq; // 对应图像帧序号
        uint64_t timestamp_ns;
    };

    ~ShmClient() = default;

    // Move-only
    ShmClient(ShmClient&&) = default;
    ShmClient& operator=(ShmClient&&) = default;
    ShmClient(const ShmClient&) = delete;
    ShmClient& operator=(const ShmClient&) = delete;

    /**
     * @brief 连接到共享内存 (消费者模式)
     *
     * 消费者连接到已由 Rust 模拟器创建的共享内存区域
     */
    [[nodiscard]] static std::expected<ShmClient, ShmError> connect() {
        // 打开元数据区域
        auto meta_result = ShmRegion::open(SHM_NAME_META, sizeof(ShmMetaRegion));
        if (!meta_result) {
            return std::unexpected(meta_result.error());
        }

        // 打开图像池
        auto pool_result = ShmRegion::open(SHM_NAME_IMAGE_POOL, IMAGE_POOL_SIZE);
        if (!pool_result) {
            return std::unexpected(pool_result.error());
        }

        // 验证魔数和版本
        const auto* meta = meta_result->as<ShmMetaRegion>();
        if (meta->header.magic != SHM_MAGIC) {
            return std::unexpected(ShmError::InvalidSize); // 实际应该是 InvalidMagic
        }
        if (meta->header.version != SHM_VERSION) {
            return std::unexpected(ShmError::InvalidSize); // 实际应该是 VersionMismatch
        }

        return ShmClient(std::move(*meta_result), std::move(*pool_result));
    }

    /**
     * @brief 创建共享内存 (生产者模式，仅用于测试)
     */
    [[nodiscard]] static std::expected<ShmClient, ShmError> create() {
        // 创建元数据区域
        auto meta_result = ShmRegion::create(SHM_NAME_META, sizeof(ShmMetaRegion));
        if (!meta_result) {
            return std::unexpected(meta_result.error());
        }

        // 创建图像池
        auto pool_result = ShmRegion::create(SHM_NAME_IMAGE_POOL, IMAGE_POOL_SIZE);
        if (!pool_result) {
            return std::unexpected(pool_result.error());
        }

        // 初始化元数据区域
        auto* meta = meta_result->as<ShmMetaRegion>();

        // 初始化头部
        meta->header.magic = SHM_MAGIC;
        meta->header.version = SHM_VERSION;
        meta->header.created_ns =
            static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        meta->header.image_width = IMAGE_WIDTH;
        meta->header.image_height = IMAGE_HEIGHT;

        // 初始化所有 TripleBuffer (CRITICAL: memset 破坏了默认初始化器)
        // 正确初始状态: state=1 (ready slot), write_idx=0, read_idx=2
        init_triple_buffer(meta->image);
        for (auto& pose: meta->poses) {
            init_triple_buffer(pose);
        }
        init_triple_buffer(meta->gimbal_cmd);

        return ShmClient(std::move(*meta_result), std::move(*pool_result));
    }

    // ============ 图像订阅 ============

    /**
     * @brief 尝试接收最新图像帧
     * @return 如果有新图像，返回 ImageFrame；否则返回 nullopt
     *
     * 返回的 cv::Mat 直接指向共享内存，零拷贝。
     * 注意：cv::Mat 在下次 recv_image() 调用后可能失效！
     */
    [[nodiscard]] std::optional<ImageFrame> recv_image() const {
        ImageOps ops(&meta_->image);
        const auto slot = ops.borrow();
        if (!slot) {
            return std::nullopt;
        }

        const auto& img_meta = **slot;

        // 计算图像数据在 pool 中的偏移
        uint8_t* img_data = image_pool_ + img_meta.buffer_id * IMAGE_SIZE;

        // 确定 OpenCV 类型
        int cv_type = CV_8UC3; // 默认 RGB8
        if (img_meta.format == 1)
            cv_type = CV_8UC3; // BGR8
        else if (img_meta.format == 2)
            cv_type = CV_8UC1; // GRAY8

        // 创建零拷贝 cv::Mat
        const cv::Mat image(
            static_cast<int>(img_meta.height),
            static_cast<int>(img_meta.width),
            cv_type,
            img_data
        );

        return ImageFrame {
            .image = image,
            .seq = img_meta.seq,
            .timestamp_ns = img_meta.timestamp_ns,
        };
    }

    /**
     * @brief 检查是否有新图像
     */
    [[nodiscard]] bool has_new_image() const {
        const ImageOps ops(&meta_->image);
        return ops.has_new_data();
    }

    // ============ 位姿订阅 ============

    /**
     * @brief 尝试接收指定类型的位姿
     * @param index 位姿类型 (POSE_GIMBAL, POSE_ODOM, POSE_MUZZLE, POSE_CAMERA,
     * POSE_CHASSIS_OBSERVATION[legacy])
     */
    [[nodiscard]] std::optional<Pose> recv_pose(const PoseIndex index) const {
        if (index > 4)
            return std::nullopt;

        PoseOps ops(&meta_->poses[index]);
        const auto slot = ops.borrow();
        if (!slot) {
            return std::nullopt;
        }

        const auto& pose = **slot;
        return Pose {
            .x = pose.position[0],
            .y = pose.position[1],
            .z = pose.position[2],
            .qw = pose.quaternion[0],
            .qx = pose.quaternion[1],
            .qy = pose.quaternion[2],
            .qz = pose.quaternion[3],
            .frame_seq = pose.frame_seq,
            .timestamp_ns = pose.timestamp_ns,
        };
    }

    /**
     * @brief 接收底盘观测数据（推荐接口）
     *
     * 数据来源为 ShmMetaRegion::chassis_observation（独立结构体通道）。
     * 若 timestamp_ns=0，表示生产者尚未发布有效数据。
     */
    [[nodiscard]] std::optional<ChassisObservation> recv_chassis_observation() const {
        const auto observation = meta_->chassis_observation;
        if (observation.timestamp_ns == 0) {
            return std::nullopt;
        }
        return observation;
    }

    // ============ Ground Truth ============

    /**
     * @brief 接收 ground truth 数据
     *
     * 数据来源为 ShmMetaRegion::ground_truth。
     * 若 timestamp_ns=0，表示生产者尚未发布有效数据。
     */
    [[nodiscard]] std::optional<GroundTruthBatch> recv_ground_truth() const {
        const auto& gt = meta_->ground_truth;
        if (gt.timestamp_ns == 0) {
            return std::nullopt;
        }
        return gt;
    }

    /**
     * @brief Receive simulator runtime state.
     *
     * The state lives in the formerly reserved tail of the metadata region, so older publishers
     * expose no valid state (`timestamp_ns == 0`) rather than an incorrect value.
     */
    [[nodiscard]] std::optional<RuntimeState> recv_runtime_state() const {
        const auto state = meta_->runtime_state;
        if (state.timestamp_ns == 0) {
            return std::nullopt;
        }
        return state;
    }

    // ============ 云台命令发布 ============

    /**
     * @brief 发布云台控制命令
     * @param yaw_deg 目标偏航角 (度)
     * @param pitch_deg 目标俯仰角 (度)
     * @param distance_m 目标距离 (米)，-1 表示无效
     * @param fire_advice 是否建议开火
     */
    void send_gimbal_cmd(
        const float yaw_deg,
        const float pitch_deg,
        const float distance_m,
        const bool fire_advice
    ) const {
        GimbalOps ops(&meta_->gimbal_cmd);
        auto& cmd = ops.borrow_mut();

        cmd.timestamp_ns =
            static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        cmd.yaw_deg = yaw_deg;
        cmd.pitch_deg = pitch_deg;
        cmd.distance_m = distance_m;
        cmd.fire_advice = fire_advice ? 1 : 0;

        ops.publish();
    }

    // ============ 相机信息 ============

    /**
     * @brief 获取相机内参 (直接访问，不使用 TripleBuffer)
     */
    [[nodiscard]] const CameraInfo& camera_info() const {
        return meta_->camera_info;
    }

    /**
     * @brief 发布位姿 (生产者模式)
     */
    void publish_pose(const PoseIndex index, const Pose& pose) const {
        if (index > 4) {
            return;
        }

        PoseOps ops(&meta_->poses[index]);
        auto& meta = ops.borrow_mut();
        meta.frame_seq = pose.frame_seq;
        meta.position[0] = static_cast<float>(pose.x);
        meta.position[1] = static_cast<float>(pose.y);
        meta.position[2] = static_cast<float>(pose.z);
        meta.quaternion[0] = static_cast<float>(pose.qw);
        meta.quaternion[1] = static_cast<float>(pose.qx);
        meta.quaternion[2] = static_cast<float>(pose.qy);
        meta.quaternion[3] = static_cast<float>(pose.qz);
        meta.timestamp_ns = pose.timestamp_ns;
        ops.publish();
    }

    /**
     * @brief 设置相机内参 (生产者模式)
     */
    void publish_camera_info(const CameraInfo& info) const {
        meta_->camera_info = info;
    }

    /**
     * @brief 发布运行时状态 (生产者模式)
     */
    void publish_runtime_state(const bool following, const uint64_t timestamp_ns) const {
        meta_->runtime_state.timestamp_ns = timestamp_ns;
        meta_->runtime_state.following = following ? 1U : 0U;
    }

    // ============ 诊断 ============

    /**
     * @brief 获取共享内存头部信息
     */
    [[nodiscard]] const ShmHeader& header() const {
        return meta_->header;
    }

    /**
     * @brief 更新心跳 (生产者调用)
     */
    void update_heartbeat() const {
        meta_->header.heartbeat_ns = now_ns();
    }

    /**
     * @brief 检查生产者是否存活 (消费者调用)
     * @param timeout_ns 超时时间 (纳秒)
     */
    [[nodiscard]] bool is_producer_alive(const uint64_t timeout_ns = 1'000'000'000) const {
        return now_ns() - meta_->header.heartbeat_ns < timeout_ns;
    }

    /**
     * @brief 等待生产者就绪
     * @param timeout 超时时间
     * @return true 如果生产者在超时前变为活跃状态
     *
     * 用于防止连接到旧的共享内存文件。调用此方法会等待直到
     * 生产者的 heartbeat 开始更新，确认是一个活跃的连接。
     */
    [[nodiscard]] bool
    wait_for_producer(const std::chrono::milliseconds timeout = std::chrono::seconds(5)) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (is_producer_alive()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    }

    // ============ 图像发布 (生产者模式，用于测试) ============

    /**
     * @brief 发布图像 (生产者模式)
     * @param image 图像数据
     * @param seq 帧序号
     * @param timestamp_ns 时间戳
     */
    void
    publish_image(const cv::Mat& image, const uint64_t seq, const uint64_t timestamp_ns) const {
        ImageOps ops(&meta_->image);
        auto& meta = ops.borrow_mut();

        // 选择下一个 buffer
        uint8_t buffer_id = meta.buffer_id;
        buffer_id = (buffer_id + 1) % 3;

        // 复制图像数据到 pool
        uint8_t* dst = image_pool_ + buffer_id * IMAGE_SIZE;
        if (image.isContinuous()) {
            std::memcpy(dst, image.data, IMAGE_SIZE);
        } else {
            for (int row = 0; row < image.rows; ++row) {
                std::memcpy(
                    dst + row * image.cols * image.channels(),
                    image.ptr(row),
                    image.cols * image.channels()
                );
            }
        }

        // 更新元数据
        meta.seq = seq;
        meta.timestamp_ns = timestamp_ns;
        meta.width = static_cast<uint32_t>(image.cols);
        meta.height = static_cast<uint32_t>(image.rows);
        meta.buffer_id = buffer_id;
        meta.format = image.type() == CV_8UC1 ? 2 : 1; // BGR8 或 GRAY8

        ops.publish();
    }

private:
    /**
     * @brief 初始化 TripleBuffer 到正确的初始状态
     *
     * ShmRegion::create() 使用 memset 清零内存，会破坏 TripleBuffer
     * 的默认成员初始化器。必须手动重新初始化。
     *
     * 正确初始状态:
     * - state = 1 (ready slot 是 1, 无 FLAG_NEW)
     * - write_idx = 0 (生产者写入 slot 0)
     * - read_idx = 2 (消费者上次读取 slot 2)
     */
    template<typename TripleBuffer>
    static void init_triple_buffer(TripleBuffer& buf) {
        buf.state.store(1, std::memory_order_relaxed);
        buf.write_idx = 0;
        buf.read_idx = 2;
    }

    /**
     * @brief 获取当前时间的纳秒时间戳 (UNIX epoch)
     *
     * 使用 system_clock 与 Rust 端的 SystemTime::now() 保持一致
     */
    [[nodiscard]] static uint64_t now_ns() {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::system_clock::now().time_since_epoch()
        )
                                         .count());
    }

    ShmClient(ShmRegion meta_region, ShmRegion pool_region):
        meta_region_(std::move(meta_region)),
        pool_region_(std::move(pool_region)),
        meta_(meta_region_.as<ShmMetaRegion>()),
        image_pool_(static_cast<uint8_t*>(pool_region_.data())) {}

    ShmRegion meta_region_;
    ShmRegion pool_region_;
    ShmMetaRegion* meta_;
    uint8_t* image_pool_;
};

} // namespace talos::ipc
