#pragma once

#include "foxglove_viz/abstract_publisher.hpp"
#include "time/time.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include <foxglove/error.hpp>
#include <foxglove/schemas.hpp>

namespace foxglove_viz
{

class EntityPublisher;

/// @brief 绑定一个 Foxglove SceneUpdateChannel 的 publisher，与一个场景绑定。
class ScenePublisher final
    : public AbstractPublisher,
      public std::enable_shared_from_this<ScenePublisher> // 让自身具备智能指针一样的可复制性
{
public:
    using Ptr = std::shared_ptr<ScenePublisher>;

    ScenePublisher(const ScenePublisher &) = delete;
    ScenePublisher &operator=(const ScenePublisher &) = delete;
    ScenePublisher(ScenePublisher &&other) = delete;
    ScenePublisher &operator=(ScenePublisher &&other) = delete;
    ~ScenePublisher() override = default;

    /// @brief 创建当前场景下的实体发布器。
    std::shared_ptr<EntityPublisher> create_entity_publisher(
        std::string_view frame_id,
        std::string_view entity_id);

    /// @brief 发布完整场景更新信息。
    bool publish(
        foxglove::schemas::SceneUpdate update,
        std::optional<timetool::Timestamp> log_time = std::nullopt);

    /// @brief 删除当前场景下的全部实体。
    bool delete_all(std::optional<timetool::Timestamp> log_time = std::nullopt);

private:
    friend class FoxgloveServer;

    /// @brief 构造 scene publisher。
    ScenePublisher(
        FoxgloveServer &server,
        std::string_view topic);

    foxglove::schemas::SceneUpdateChannel m_channel;
};

/// @brief 绑定于某个场景的实体 publisher，具备固定的 frame_id 和 id。
///
/// EntityPublisher 不注册 Foxglove topic，只通过所属 ScenePublisher 发布 SceneEntity。
class EntityPublisher final
{
public:
    using Ptr = std::shared_ptr<EntityPublisher>;

    /// @brief 构造 entity publisher。
    EntityPublisher(
        std::shared_ptr<ScenePublisher> scene,
        std::string_view frame_id,
        std::string_view entity_id);

    EntityPublisher(const EntityPublisher &) = delete;
    EntityPublisher &operator=(const EntityPublisher &) = delete;
    EntityPublisher(EntityPublisher &&other) = delete;
    EntityPublisher &operator=(EntityPublisher &&other) = delete;
    ~EntityPublisher() = default;

    /// @brief 获取所属 Foxglove topic。
    std::string_view topic() const noexcept;

    /// @brief 获取发布几何体使用的 frame_id。
    std::string_view frame_id() const noexcept;

    /// @brief 获取发布几何体使用的 entity_id。
    std::string_view entity_id() const noexcept;

    /// @brief 发布一个 SceneEntity，空 frame_id/id 会自动填入当前 entity 信息。
    bool publish_entity(
        foxglove::schemas::SceneEntity entity,
        std::optional<timetool::Timestamp> log_time = std::nullopt);

    /// @brief 发布一组 cube primitive。
    bool publish_cubes(
        std::span<const foxglove::schemas::CubePrimitive> cubes,
        std::optional<timetool::Timestamp> log_time = std::nullopt);

    /// @brief 发布一组 sphere primitive。
    bool publish_spheres(
        std::span<const foxglove::schemas::SpherePrimitive> spheres,
        std::optional<timetool::Timestamp> log_time = std::nullopt);

    /// @brief 发布一组 arrow primitive。
    bool publish_arrows(
        std::span<const foxglove::schemas::ArrowPrimitive> arrows,
        std::optional<timetool::Timestamp> log_time = std::nullopt);

    /// @brief 删除当前 entity_id 对应的实体。
    bool delete_entity(std::optional<timetool::Timestamp> log_time = std::nullopt);

private:
    std::shared_ptr<ScenePublisher> m_scene;
    std::string m_frame_id;
    std::string m_entity_id;
};

namespace detail
{

/// @brief 构造并校验 scene publisher 基础元信息。
PublisherMetaInfo make_scene_publisher_meta_info(std::string_view topic);

/// @brief 校验 entity 的 frame_id 与 entity_id。
void validate_entity_info(
    std::string_view frame_id,
    std::string_view entity_id);

/// @brief 创建 Foxglove SceneUpdateChannel。
foxglove::schemas::SceneUpdateChannel create_scene_update_channel(std::string_view topic);

} // namespace detail

} // namespace foxglove_viz
