#include "foxglove_viz/abstract_publisher.hpp"

#include <utility>

namespace foxglove_viz
{

/**
 * @brief 保存 publisher 元信息。
 */
AbstractPublisher::AbstractPublisher(
    FoxgloveServer &server,
    PublisherMetaInfo info)
    : m_server(server),
      m_info(std::move(info))
{
}

/**
 * @brief 纯虚析构函数定义。
 */
AbstractPublisher::~AbstractPublisher()
{
}

/**
 * @brief 返回 publisher topic。
 */
std::string_view AbstractPublisher::topic() const noexcept
{
    return m_info.topic;
}

/**
 * @brief 返回 publisher 基础元信息。
 */
const PublisherMetaInfo &AbstractPublisher::info() const noexcept
{
    return m_info;
}

/**
 * @brief 返回所属 server。
 */
FoxgloveServer &AbstractPublisher::server() noexcept
{
    return m_server;
}

} // namespace foxglove_viz
