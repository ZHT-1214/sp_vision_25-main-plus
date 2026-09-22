#pragma once

#include "foxglove_viz/foxglove_server.hpp"

#include <string_view>

namespace foxglove_viz
{

/// @brief 非模板 publisher 基类，保存 topic 与 publisher 类型元信息。
///
/// 派生类负责具体消息内容、底层 channel 发布以及自身生命周期对应的注册策略。
class AbstractPublisher
{
public:
    /// @brief 纯虚析构函数，使该类保持抽象基类语义。
    virtual ~AbstractPublisher() = 0;

    AbstractPublisher(const AbstractPublisher &) = delete;
    AbstractPublisher &operator=(const AbstractPublisher &) = delete;
    AbstractPublisher(AbstractPublisher &&other) = delete;
    AbstractPublisher &operator=(AbstractPublisher &&other) = delete;

    /// @brief 获取 publisher 的 topic。
    /// @return topic 字符串视图。
    std::string_view topic() const noexcept;

    /// @brief 获取 publisher 基础元信息。
    /// @return publisher 基础元信息引用。
    const PublisherMetaInfo &info() const noexcept;

protected:
    AbstractPublisher(
        FoxgloveServer &server,
        PublisherMetaInfo info);

    /// @brief 获取所属 FoxgloveServer。
    FoxgloveServer &server() noexcept;

private:
    FoxgloveServer &m_server;
    PublisherMetaInfo m_info;
};

} // namespace foxglove_viz
