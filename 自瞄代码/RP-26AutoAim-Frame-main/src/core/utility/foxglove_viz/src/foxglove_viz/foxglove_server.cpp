#include "foxglove_viz/abstract_publisher.hpp"
#include "foxglove_viz/geometry_publisher.hpp"

#include <stdexcept>
#include <utility>

#include <foxglove/foxglove.hpp>

namespace foxglove_viz
{

/**
 * @brief 创建底层 Foxglove WebSocket server。
 */
FoxgloveServer::FoxgloveServer(std::string host, uint16_t port)
{
    foxglove::setLogLevel(foxglove::LogLevel::Warn);

    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = std::move(host);
    ws_options.port = port;

    auto server_result = foxglove::WebSocketServer::create(std::move(ws_options));
    if (!server_result.has_value())
        throw std::runtime_error(
            std::string("failed to create Foxglove WebSocket server: ") +
            foxglove::strerror(server_result.error()));

    m_server_ptr = std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value()));
}

/**
 * @brief 将 data publisher 记录到本地注册表。
 */
void FoxgloveServer::register_data_publisher(const AbstractPublisher &publisher)
{
    std::lock_guard<std::mutex> lock(m_publishers_mutex);

    const std::string topic(publisher.topic());
    if (m_publishers.contains(topic))
        throw std::runtime_error("foxglove topic already registered: " + topic);

    m_publishers.emplace(topic, publisher.info());
}

std::shared_ptr<ScenePublisher> FoxgloveServer::create_scene_publisher(
    std::string_view topic)
{
    std::lock_guard<std::mutex> lock(m_publishers_mutex);

    const std::string topic_string(topic);
    const auto publisher_it = m_publishers.find(topic_string);
    if (publisher_it != m_publishers.end())
    {
        // 如果话题已经注册为 DataPublisher，异常
        if (publisher_it->second.kind != PublisherMetaInfo::PublisherKind::Scene)
            throw std::runtime_error("foxglove topic already registered as non-scene: " + topic_string);

        const auto scene_it = m_scenes.find(topic_string);
        if (scene_it != m_scenes.end())
            return scene_it->second;

        throw std::runtime_error(
            "internal foxglove_viz error: scene publisher registry is inconsistent for topic '" +
            topic_string +
            "'. Please report this as a bug.");
    }

    // 若不存在则创建新的；scene topic 的注册信息由 FoxgloveServer 统一维护。
    std::shared_ptr<ScenePublisher> scene(new ScenePublisher(*this, topic));

    const std::string scene_topic(scene->topic());
    m_publishers.emplace(scene_topic, scene->info());
    m_scenes.emplace(scene_topic, scene);
    return scene;
}

std::shared_ptr<EntityPublisher> FoxgloveServer::create_entity_publisher(
    std::string_view topic,
    std::string_view frame_id,
    std::string_view entity_id)
{
    return create_scene_publisher(topic)->create_entity_publisher(frame_id, entity_id);
}

/**
 * @brief 从本地注册表移除 data publisher。
 */
void FoxgloveServer::unregister_data_publisher(std::string_view topic) noexcept
{
    std::lock_guard<std::mutex> lock(m_publishers_mutex);
    const std::string topic_string(topic);
    m_publishers.erase(topic_string);
}

/**
 * @brief 查询 topic 是否已经存在本地注册记录。
 */
bool FoxgloveServer::has_publisher(std::string_view topic) const
{
    std::lock_guard<std::mutex> lock(m_publishers_mutex);
    return m_publishers.contains(std::string(topic));
}

/**
 * @brief 获取单个 topic 的本地注册元信息。
 */
std::optional<PublisherMetaInfo> FoxgloveServer::get_publisher_info(std::string_view topic) const
{
    std::lock_guard<std::mutex> lock(m_publishers_mutex);

    const auto it = m_publishers.find(std::string(topic));
    if (it == m_publishers.end())
        return std::nullopt;

    return it->second;
}

/**
 * @brief 获取全部 publisher 本地注册元信息。
 */
std::vector<PublisherMetaInfo> FoxgloveServer::get_publisher_infos() const
{
    std::lock_guard<std::mutex> lock(m_publishers_mutex);

    std::vector<PublisherMetaInfo> infos;
    infos.reserve(m_publishers.size());
    for (const auto &[topic, info] : m_publishers)
        infos.push_back(info);

    return infos;
}

/**
 * @brief 返回 WebSocket server 实际监听端口。
 */
uint16_t FoxgloveServer::port() const noexcept
{
    return m_server_ptr ? m_server_ptr->port() : 0;
}

/**
 * @brief 返回进程内唯一的 FoxgloveServer。
 */
FoxgloveServer &global_foxglove_server()
{
    static FoxgloveServer server;
    return server;
}

} // namespace foxglove_viz
