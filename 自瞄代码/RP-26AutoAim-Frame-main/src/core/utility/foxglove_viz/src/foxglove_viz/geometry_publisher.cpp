#include "foxglove_viz/geometry_publisher.hpp"

#include <stdexcept>
#include <utility>

#include <foxglove/foxglove.hpp>

namespace foxglove_viz
{

std::optional<uint64_t> to_log_time_ns(std::optional<timetool::Timestamp> log_time)
{
    if (!log_time)
        return std::nullopt;
    else
        return timetool::to_epoch_nanoseconds(*log_time);
}

ScenePublisher::ScenePublisher(
    FoxgloveServer &server,
    std::string_view topic)
    : AbstractPublisher(server, detail::make_scene_publisher_meta_info(topic)),
      m_channel(detail::create_scene_update_channel(this->topic()))
{
}

std::shared_ptr<EntityPublisher> ScenePublisher::create_entity_publisher(
    std::string_view frame_id,
    std::string_view entity_id)
{
    return std::make_shared<EntityPublisher>(shared_from_this(), frame_id, entity_id);
}

bool ScenePublisher::publish(
    foxglove::schemas::SceneUpdate update,
    std::optional<timetool::Timestamp> log_time)
{
    const foxglove::FoxgloveError error = m_channel.log(update, to_log_time_ns(log_time));
    return error == foxglove::FoxgloveError::Ok;
}

bool ScenePublisher::delete_all(std::optional<timetool::Timestamp> log_time)
{
    foxglove::schemas::SceneEntityDeletion deletion;
    deletion.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL;

    foxglove::schemas::SceneUpdate update;
    update.deletions.push_back(std::move(deletion));
    return publish(std::move(update), log_time);
}

EntityPublisher::EntityPublisher(
    std::shared_ptr<ScenePublisher> scene,
    std::string_view frame_id,
    std::string_view entity_id)
    : m_scene(std::move(scene)),
      m_frame_id(frame_id),
      m_entity_id(entity_id)
{
    if (!m_scene)
        throw std::invalid_argument("foxglove geometry scene publisher must not be null");

    detail::validate_entity_info(m_frame_id, m_entity_id);
}

std::string_view EntityPublisher::topic() const noexcept
{
    return m_scene->topic();
}

std::string_view EntityPublisher::frame_id() const noexcept
{
    return m_frame_id;
}

std::string_view EntityPublisher::entity_id() const noexcept
{
    return m_entity_id;
}

bool EntityPublisher::publish_entity(
    foxglove::schemas::SceneEntity entity,
    std::optional<timetool::Timestamp> log_time)
{
    if (entity.frame_id.empty())
        entity.frame_id = m_frame_id;
    if (entity.id.empty())
        entity.id = m_entity_id;

    foxglove::schemas::SceneUpdate update;
    update.entities.push_back(std::move(entity));
    return m_scene->publish(std::move(update), log_time);
}

bool EntityPublisher::publish_cubes(
    std::span<const foxglove::schemas::CubePrimitive> cubes,
    std::optional<timetool::Timestamp> log_time)
{
    foxglove::schemas::SceneEntity entity;
    entity.frame_id = m_frame_id;
    entity.id = m_entity_id;
    entity.cubes.assign(cubes.begin(), cubes.end());
    return publish_entity(std::move(entity), log_time);
}

bool EntityPublisher::publish_spheres(
    std::span<const foxglove::schemas::SpherePrimitive> spheres,
    std::optional<timetool::Timestamp> log_time)
{
    foxglove::schemas::SceneEntity entity;
    entity.frame_id = m_frame_id;
    entity.id = m_entity_id;
    entity.spheres.assign(spheres.begin(), spheres.end());
    return publish_entity(std::move(entity), log_time);
}

bool EntityPublisher::publish_arrows(
    std::span<const foxglove::schemas::ArrowPrimitive> arrows,
    std::optional<timetool::Timestamp> log_time)
{
    foxglove::schemas::SceneEntity entity;
    entity.frame_id = m_frame_id;
    entity.id = m_entity_id;
    entity.arrows.assign(arrows.begin(), arrows.end());
    return publish_entity(std::move(entity), log_time);
}

bool EntityPublisher::delete_entity(std::optional<timetool::Timestamp> log_time)
{
    foxglove::schemas::SceneEntityDeletion deletion;
    deletion.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID;
    deletion.id = m_entity_id;

    foxglove::schemas::SceneUpdate update;
    update.deletions.push_back(std::move(deletion));
    return m_scene->publish(std::move(update), log_time);
}

namespace detail
{

PublisherMetaInfo make_scene_publisher_meta_info(std::string_view topic)
{
    if (topic.empty())
        throw std::invalid_argument("foxglove topic must not be empty");

    PublisherMetaInfo info;
    info.topic = std::string(topic);
    info.kind = PublisherMetaInfo::PublisherKind::Scene;
    return info;
}

void validate_entity_info(
    std::string_view frame_id,
    std::string_view entity_id)
{
    if (frame_id.empty())
        throw std::invalid_argument("foxglove geometry frame_id must not be empty");

    if (entity_id.empty())
        throw std::invalid_argument("foxglove geometry entity_id must not be empty");
}

foxglove::schemas::SceneUpdateChannel create_scene_update_channel(std::string_view topic)
{
    auto channel_result = foxglove::schemas::SceneUpdateChannel::create(topic);
    if (!channel_result.has_value())
        throw std::runtime_error(
            std::string("failed to create Foxglove scene update channel: ") +
            foxglove::strerror(channel_result.error()));

    return std::move(channel_result.value());
}

} // namespace detail
} // namespace foxglove_viz
