#pragma once

#include "foxglove_viz/types.hpp"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <foxglove/server.hpp>

namespace foxglove_viz
{

class AbstractPublisher;

/// @brief 发布指定字段类型列表的 data publisher。
/// @tparam Ts 字段类型列表；支持 double、bool、int64_t、uint64_t 及这些类型的 std::vector/std::array。
template <FieldType... Ts>
class DataPublisher; // 类声明不需要指定派生关系

class ScenePublisher;
class EntityPublisher;

/// @brief 为 Foxglove 发布提供支持。
/// 通过 global_foxglove_server() 获取实例，再调用 create_publisher() 创建
/// topic 发布器。data topic 只能创建一个 DataPublisher；scene topic
/// 可以复用同一个 ScenePublisher 创建多个 entity publisher。
class FoxgloveServer
{
    friend FoxgloveServer &global_foxglove_server();

    template <FieldType... Ts>
    friend class DataPublisher;

public:
    ~FoxgloveServer() = default;

    FoxgloveServer(const FoxgloveServer &) = delete;
    FoxgloveServer &operator=(const FoxgloveServer &) = delete;
    FoxgloveServer(FoxgloveServer &&) = delete;
    FoxgloveServer &operator=(FoxgloveServer &&) = delete;

    // 创建 publisher，字段名自动生成。
    // 单字段默认名为 value，多字段默认名为 value0、value1 ...
    //
    // 用法：
    // auto pub = foxglove_viz::global_foxglove_server().create_publisher<double>("/speed");
    // pub->publish(12.3);
    template <FieldType... Ts>
    typename DataPublisher<Ts...>::Ptr create_publisher(std::string_view topic);

    // 创建 publisher，并用大括号直接指定字段名。
    // 这是最常用的写法，字段名数量必须和模板参数数量一致。
    //
    // auto pub = foxglove_viz::global_foxglove_server().create_publisher<double, double>("/gimbal", {"yaw", "pitch"});
    // pub->publish(10.0, -2.5);
    template <FieldType... Ts>
    typename DataPublisher<Ts...>::Ptr create_publisher(
        std::string_view topic,
        std::initializer_list<std::string_view> field_names);

    // 创建 publisher，并使用 std::array 指定字段名。
    // 适合字段名已经在别处整理成数组的场景。
    //
    // constexpr std::array<std::string_view, 3> fields = {"x", "count", "ok"};
    // auto pub = foxglove_viz::global_foxglove_server().create_publisher<double, int64_t, bool>("/demo", fields);
    // pub->publish(1.2, int64_t{3}, true);
    template <FieldType... Ts>
    typename DataPublisher<Ts...>::Ptr create_publisher(
        std::string_view topic,
        std::array<std::string_view, sizeof...(Ts)> field_names);

    // 创建 scene publisher。
    // 一个 scene publisher 对应一个 Foxglove SceneUpdate topic，可在其下继续
    // 创建多个 entity publisher。
    //
    // auto scene = foxglove_viz::global_foxglove_server()
    //     .create_scene_publisher("/scene_car");
    // auto armor = scene->create_entity_publisher("car_frame", "armors_3d");
    // armor->publish_cubes(cubes);
    std::shared_ptr<ScenePublisher> create_scene_publisher(std::string_view topic);

    // 创建 entity publisher。
    // 如果 topic 对应的 scene publisher 已存在，则复用它；否则自动创建。
    // 因此多个模块可以通过同一个 topic 发布不同 entity。
    //
    // auto pub = foxglove_viz::global_foxglove_server()
    //     .create_entity_publisher("/scene_car", "car_frame", "armors_3d");
    // pub->publish_cubes(cubes);
    std::shared_ptr<EntityPublisher> create_entity_publisher(
        std::string_view topic,
        std::string_view frame_id,
        std::string_view entity_id);

    /// @brief 判断 topic 是否已有 publisher 注册。
    /// @return 已注册返回 true，否则返回 false。
    bool has_publisher(std::string_view topic) const;

    /// @brief 查询指定 topic 的 publisher 元信息。
    /// @return topic 已注册时返回元信息，否则返回 std::nullopt。
    std::optional<PublisherMetaInfo> get_publisher_info(std::string_view topic) const;

    /// @brief 获取当前所有已注册 publisher 的元信息。
    std::vector<PublisherMetaInfo> get_publisher_infos() const;

    /// @brief 获取 Foxglove 连接端口。
    uint16_t port() const noexcept;

private:
    explicit FoxgloveServer(std::string host = "0.0.0.0", uint16_t port = 8765);

    /// @brief 记录 data publisher。
    void register_data_publisher(const AbstractPublisher &publisher);

    /// @brief 移除 data publisher 记录。
    void unregister_data_publisher(std::string_view topic) noexcept;

    /// @brief Foxglove server 实例。
    std::unique_ptr<foxglove::WebSocketServer> m_server_ptr;

    mutable std::mutex m_publishers_mutex;

    /// @brief 已创建 publisher 的元信息。
    std::unordered_map<std::string, PublisherMetaInfo> m_publishers;

    /// @brief 已创建的 scene publisher，用于复用同一 topic。
    std::unordered_map<std::string, std::shared_ptr<ScenePublisher>> m_scenes;
};

/// @brief 获取全局 FoxgloveServer。
/// @return FoxgloveServer 引用。
FoxgloveServer &global_foxglove_server();

} // namespace foxglove_viz
