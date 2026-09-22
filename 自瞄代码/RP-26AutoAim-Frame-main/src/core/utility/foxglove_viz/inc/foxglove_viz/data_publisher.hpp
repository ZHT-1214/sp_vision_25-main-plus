#pragma once

#include "foxglove_viz/abstract_publisher.hpp"
#include "time/time.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/error.hpp>

namespace foxglove_viz
{

/// @brief 一个字段的元信息，包含名字与类型信息
struct FieldMetaInfo
{
    /// @brief Foxglove JSON schema 中支持的基础字段类型。
    enum class BasicTypeID
    {
        Number,
        Integer,
        Boolean,
    };

    /// @brief 字段名。
    std::string name;

    /// @brief 标量字段类型；数组字段使用数组元素类型。
    BasicTypeID type;

    /// @brief 当前字段是否为 JSON array。
    bool is_array = false;

    /// @brief std::array 字段的固定长度；std::vector 字段为空。
    std::optional<std::size_t> fixed_array_size = std::nullopt;
};

namespace detail
{

template <typename T>
struct FieldTypeTraits
{
    using ElementType = T;
    static constexpr bool is_array = false;
    static constexpr bool has_fixed_array_size = false;
    static constexpr std::size_t fixed_array_size = 0;
};

template <typename T, typename Allocator>
struct FieldTypeTraits<std::vector<T, Allocator>>
{
    using ElementType = T;
    static constexpr bool is_array = true;
    static constexpr bool has_fixed_array_size = false;
    static constexpr std::size_t fixed_array_size = 0;
};

template <typename T, std::size_t N>
struct FieldTypeTraits<std::array<T, N>>
{
    using ElementType = T;
    static constexpr bool is_array = true;
    static constexpr bool has_fixed_array_size = true;
    static constexpr std::size_t fixed_array_size = N;
};

/// @brief 将 C++ 标量字段类型映射为 FieldMetaInfo::BasicTypeID
/// @tparam T C++ 标量字段类型。
/// @return 对应的字段类型 ID。
template <FieldBasicType T>
consteval FieldMetaInfo::BasicTypeID classify_basic_type()
{
    if constexpr (std::same_as<T, double>)
        return FieldMetaInfo::BasicTypeID::Number;
    else if constexpr (std::same_as<T, int64_t> || std::same_as<T, uint64_t>)
        return FieldMetaInfo::BasicTypeID::Integer;
    else
        return FieldMetaInfo::BasicTypeID::Boolean;
}

/// @brief 根据 C++ 字段类型填充部分字段元信息。
/// @tparam T C++ 字段类型。
/// @return 字段元信息，字段名由 make_data_publisher_fields 填入。
template <FieldType T>
FieldMetaInfo classify_type()
{
    using Traits = FieldTypeTraits<T>;

    FieldMetaInfo info;
    info.type = classify_basic_type<typename Traits::ElementType>();
    info.is_array = Traits::is_array;
    if constexpr (Traits::has_fixed_array_size)
        info.fixed_array_size = Traits::fixed_array_size;
    return info;
}

/// @brief 检查 publish 实参类型是否匹配 publisher 模板字段类型的主模板。
// 主模板：具有两个模板形参
template <typename ExpectedTuple, typename PassedTuple>
struct PublishArgsMatch;

// 偏特化模板：其实参列表与主模板的形参列表符合，并具备自己的形参列表
template <typename... Expected, typename... Passed>
struct PublishArgsMatch<std::tuple<Expected...>, std::tuple<Passed...>>
{
    // 使用原地 lambda 表达式计算
    static constexpr bool value = [] {
        if constexpr (sizeof...(Expected) != sizeof...(Passed))
            return false;
        else
        {
            // 尝试用std::make_index_sequence<N>去匹配std::index_sequence<Is...>这一模板类型，匹配成功后推导出模板参数<Is...>；
            // 本质上是模板参数推导语法：当不显式指定模板参数的时候，编译器会尝试使用形参的模板类型来匹配实参的类型，最后推导出模板类型是什么
            constexpr auto is_tuple_match = []<std::size_t... Is>(std::index_sequence<Is...>)->bool {
                // 最后推导出 <std::size_t... Is> 为 <0, 1, 2, 3, ..., sizeof...(Expected) - 1>
                return (
                    std::same_as<
                        std::tuple_element_t<Is, std::tuple<Expected...>>,
                        std::remove_cvref_t<std::tuple_element_t<Is, std::tuple<Passed...>>>> &&
                    ...);
            };
            return is_tuple_match(std::make_index_sequence<sizeof...(Expected)>{});
        }
    }();
};

/// @brief 构造并校验 data publisher 字段元信息。
/// @param topic Foxglove topic。
/// @param field_names 字段名列表。
/// @param fields 字段元信息列表。
/// @return 字段元信息列表。
std::vector<FieldMetaInfo> make_field_meta_info(
    std::string_view topic,
    const std::vector<std::string> &field_names,
    std::vector<FieldMetaInfo> fields);

/// @brief 生成默认字段名。
/// @param field_count 字段数量。
/// @return 字段名列表。
std::vector<std::string> make_default_field_names(std::size_t field_count);

/// @brief 根据 data publisher 字段元信息生成 JSON schema。
/// @param fields 字段元信息列表。
/// @return JSON schema 字符串。
std::string make_json_schema(const std::vector<FieldMetaInfo> &fields);

/// @brief 创建 Foxglove RawChannel。
/// @param topic Foxglove topic。
/// @param schema_data JSON schema 字符串，调用期间必须有效。
/// @return Foxglove RawChannel。
foxglove::RawChannel create_raw_channel(
    std::string_view topic,
    const std::string &schema_data);

/// @brief 转义 JSON 字符串内容。
/// @param text 原始文本。
/// @return JSON 转义后的文本。
std::string escape_json(std::string_view text);

/// @brief 向 JSON 字符串追加浮点值。
void append_json_value(std::string &json, double value);

/// @brief 向 JSON 字符串追加布尔值。
void append_json_value(std::string &json, bool value);

/// @brief 向 JSON 字符串追加有符号整数值。
void append_json_value(std::string &json, int64_t value);

/// @brief 向 JSON 字符串追加无符号整数值。
void append_json_value(std::string &json, uint64_t value);

/// @brief 向 JSON 字符串追加数组值。
/// @tparam Range 数组容器类型。
/// @param json JSON 字符串。
/// @param values 数组值。
template <typename Range>
void append_json_array(std::string &json, const Range &values)
{
    json.push_back('[');

    bool is_first = true;
    for (const auto value : values)
    {
        if (!is_first)
            json.push_back(',');

        append_json_value(json, value);
        is_first = false;
    }

    json.push_back(']');
}

/// @brief 向 JSON 字符串追加 std::vector 数组值。
template <FieldBasicType T, typename Allocator>
void append_json_value(std::string &json, const std::vector<T, Allocator> &values)
{
    append_json_array(json, values);
}

/// @brief 向 JSON 字符串追加 std::array 数组值。
template <FieldBasicType T, std::size_t N>
void append_json_value(std::string &json, const std::array<T, N> &values)
{
    append_json_array(json, values);
}

/// @brief 将字段名和值序列化为 JSON object 字符串。
/// @tparam Ts 值类型列表。
/// @param fields 字段元信息。
/// @param values 字段值列表。
/// @return JSON object 字符串。
template <typename... Ts>
std::string make_json(const std::vector<FieldMetaInfo> &fields, Ts &&...values)
{
    std::string json;
    json.reserve(fields.size() * 24 + 2);
    json.push_back('{');

    std::size_t index = 0;
    auto append_field = [&](auto &&value) {
        if (index != 0)
            json.push_back(',');

        json.push_back('"');
        json += escape_json(fields[index].name);
        json += "\":";
        append_json_value(json, std::forward<decltype(value)>(value));
        ++index;
    };

    (append_field(std::forward<Ts>(values)), ...);
    json.push_back('}');
    return json;
}

} // namespace detail

template <FieldType... Ts>
class DataPublisher final : public AbstractPublisher
{
    static_assert(sizeof...(Ts) > 0, "DataPublisher must publish at least one field");

public:
    using Ptr = std::shared_ptr<DataPublisher<Ts...>>;

    /// @brief 构造 publisher。
    DataPublisher(
        FoxgloveServer &server,
        std::string_view topic,
        std::vector<std::string> field_names)
        : DataPublisher(
              server,
              topic,
              detail::make_field_meta_info(
                  topic,
                  field_names, // 这里把字段名字加入到字段元信息之中
                  std::vector<FieldMetaInfo>{detail::classify_type<Ts>()...} // 传入参数包 Ts，生成结果包，然后用...展开
                )
            )
    {
    }

    DataPublisher(const DataPublisher &) = delete;
    DataPublisher &operator=(const DataPublisher &) = delete;
    DataPublisher(DataPublisher &&other) = delete;
    DataPublisher &operator=(DataPublisher &&other) = delete;
    ~DataPublisher() override
    {
        m_channel.reset();
        unregister_from_server();
    }

    /// @brief 获取字段元信息。
    /// @return 字段元信息列表引用。
    const std::vector<FieldMetaInfo> &fields() const noexcept
    {
        return m_fields;
    }

    /// @brief 发布一条消息到 Foxglove topic。
    /// @tparam Us 实参类型列表，去掉 cv/ref 后必须与 Ts... 一一相同。
    /// @param values 字段值，数量和类型必须与 create_publisher 的模板参数一致。
    /// @return 发布成功返回 true，否则返回 false。
    template <typename... Us>
    bool publish(Us &&...values)
    {
        return publish_impl(std::nullopt, std::forward<Us>(values)...);
    }

    /// @brief 使用指定时间戳发布一条消息到 Foxglove topic。
    /// @tparam Us 实参类型列表，去掉 cv/ref 后必须与 Ts... 一一相同。
    /// @param log_time 消息时间戳。
    /// @param values 字段值，数量和类型必须与 create_publisher 的模板参数一致。
    /// @return 发布成功返回 true，否则返回 false。
    template <typename... Us>
    bool publish_with_time(timetool::Timestamp log_time, Us &&...values)
    {
        return publish_impl(
            timetool::to_epoch_nanoseconds(log_time),
            std::forward<Us>(values)...);
    }

private:
    /// @brief 将当前 data publisher 注册到所属 server。
    void register_to_server()
    {
        server().register_data_publisher(*this);
    }

    /// @brief 从所属 server 注销当前 data publisher。
    void unregister_from_server() noexcept
    {
        server().unregister_data_publisher(topic());
    }

    /// @brief 注册 data topic 并创建 RawChannel；创建失败时回滚注册信息。
    static foxglove::RawChannel reg_info_and_create_channel(
        DataPublisher &publisher,
        const std::vector<FieldMetaInfo> &fields)
    {
        publisher.register_to_server();

        try
        {
            return detail::create_raw_channel(
                publisher.topic(),
                detail::make_json_schema(fields));
        }
        catch (...)
        {
            publisher.unregister_from_server();
            throw;
        }
    }

    template <typename... Us>
    bool publish_impl(std::optional<uint64_t> log_time, Us &&...values)
    {
        static_assert(
            detail::PublishArgsMatch<std::tuple<Ts...>, std::tuple<Us...>>::value,
            "DataPublisher publish arguments must exactly match the publisher template "
            "parameters after removing cv/ref qualifiers");

        const std::string json = detail::make_json(fields(), std::forward<Us>(values)...);
        const foxglove::FoxgloveError error = m_channel->log(
            reinterpret_cast<const std::byte *>(json.data()),
            json.size(),
            log_time);
        return error == foxglove::FoxgloveError::Ok;
    }

    DataPublisher(
        FoxgloveServer &server,
        std::string_view topic,
        std::vector<FieldMetaInfo> fields)
        : AbstractPublisher(
              server,
              PublisherMetaInfo{
                  .topic = std::string(topic),
                  .kind = PublisherMetaInfo::PublisherKind::Data,
              }),
          m_fields(std::move(fields)),
          m_channel(reg_info_and_create_channel(*this, m_fields))
    {
    }

    std::vector<FieldMetaInfo> m_fields;
    std::optional<foxglove::RawChannel> m_channel;
};

/// @brief 创建 publisher，并使用默认字段名。
template <FieldType... Ts>
typename DataPublisher<Ts...>::Ptr FoxgloveServer::create_publisher(std::string_view topic)
{
    return std::make_shared<DataPublisher<Ts...>>(
        *this,
        topic,
        detail::make_default_field_names(sizeof...(Ts)));
}

/// @brief 创建 publisher，并使用 initializer_list 字段名。
template <FieldType... Ts>
typename DataPublisher<Ts...>::Ptr FoxgloveServer::create_publisher(
    std::string_view topic,
    std::initializer_list<std::string_view> field_names)
{
    std::vector<std::string> names;
    names.reserve(field_names.size());
    for (std::string_view field_name : field_names)
        names.emplace_back(field_name);

    return std::make_shared<DataPublisher<Ts...>>(
        *this,
        topic,
        std::move(names));
}

/// @brief 创建 publisher，并使用 array 字段名。
template <FieldType... Ts>
typename DataPublisher<Ts...>::Ptr FoxgloveServer::create_publisher(
    std::string_view topic,
    std::array<std::string_view, sizeof...(Ts)> field_names)
{
    std::vector<std::string> names;
    names.reserve(field_names.size());
    for (std::string_view field_name : field_names)
        names.emplace_back(field_name);

    return std::make_shared<DataPublisher<Ts...>>(
        *this,
        topic,
        std::move(names));
}

} // namespace foxglove_viz
