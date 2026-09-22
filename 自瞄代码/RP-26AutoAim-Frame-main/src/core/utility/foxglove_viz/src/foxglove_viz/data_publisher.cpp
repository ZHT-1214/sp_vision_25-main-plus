#include "foxglove_viz/data_publisher.hpp"

#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include <foxglove/foxglove.hpp>

namespace foxglove_viz
{

/**
 * @brief 将字段类型枚举转换为 JSON schema 类型名。
 */
std::string field_type_to_schema_type(FieldMetaInfo::BasicTypeID type)
{
    switch (type)
    {
    case FieldMetaInfo::BasicTypeID::Number:
        return "number";
    case FieldMetaInfo::BasicTypeID::Integer:
        return "integer";
    case FieldMetaInfo::BasicTypeID::Boolean:
        return "boolean";
    }

    throw std::logic_error("unknown FoxgloveFieldType");
}

namespace detail
{

/**
 * @brief 构造 data publisher 字段元信息并执行基本校验。
 */
std::vector<FieldMetaInfo> make_field_meta_info(
    std::string_view topic,
    const std::vector<std::string> &field_names,
    std::vector<FieldMetaInfo> fields)
{
    if (topic.empty())
        throw std::invalid_argument("foxglove topic must not be empty");

    if (field_names.size() != fields.size())
        throw std::invalid_argument("field name count must match field type count");

    std::unordered_set<std::string_view> names_set;
    names_set.reserve(field_names.size());
    for (const std::string &name : field_names)
    {
        if (name.empty())
            throw std::invalid_argument("foxglove field name must not be empty");

        if (!names_set.insert(name).second)
            throw std::invalid_argument("foxglove field names must be unique");
    }

    for (std::size_t i = 0; i < field_names.size(); ++i)
        fields[i].name = field_names[i];

    return fields;
}

/**
 * @brief 生成默认字段名。
 */
std::vector<std::string> make_default_field_names(std::size_t field_count)
{
    std::vector<std::string> field_names;
    field_names.reserve(field_count);

    if (field_count == 1)
    {
        field_names.emplace_back("value");
        return field_names;
    }

    for (std::size_t i = 0; i < field_count; ++i)
        field_names.emplace_back("value" + std::to_string(i));

    return field_names;
}

/**
 * @brief 对字段名进行 JSON 字符串转义。
 */
std::string escape_json(std::string_view text)
{
    std::string escaped;
    escaped.reserve(text.size());

    for (char c : text)
        switch (c)
        {
        case '"':
            escaped += "\\\"";
            break;
        case '\\':
            escaped += "\\\\";
            break;
        case '\b':
            escaped += "\\b";
            break;
        case '\f':
            escaped += "\\f";
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20)
            {
                std::ostringstream oss;
                oss << "\\u"
                    << std::hex
                    << std::setw(4)
                    << std::setfill('0')
                    << static_cast<int>(static_cast<unsigned char>(c));
                escaped += oss.str();
            }
            else
                escaped.push_back(c);
            break;
        }

    return escaped;
}

/**
 * @brief 生成 RawChannel 使用的 JSON schema。
 */
std::string make_json_schema(const std::vector<FieldMetaInfo> &fields)
{
    std::string schema;
    schema += R"({"type":"object","properties":{)";

    for (std::size_t i = 0; i < fields.size(); ++i)
    {
        if (i != 0)
            schema.push_back(',');

        schema.push_back('"');
        schema += escape_json(fields[i].name);
        if (fields[i].is_array)
        {
            schema += R"(":{"type":"array","items":{"type":")";
            schema += field_type_to_schema_type(fields[i].type);
            schema += R"("})";
            if (fields[i].fixed_array_size.has_value())
            {
                const std::string array_size = std::to_string(*fields[i].fixed_array_size);
                schema += R"(,"minItems":)";
                schema += array_size;
                schema += R"(,"maxItems":)";
                schema += array_size;
            }
            schema.push_back('}');
        }
        else
        {
            schema += R"(":{"type":")";
            schema += field_type_to_schema_type(fields[i].type);
            schema += R"("})";
        }
    }

    schema += R"(},"required":[)";
    for (std::size_t i = 0; i < fields.size(); ++i)
    {
        if (i != 0)
            schema.push_back(',');

        schema.push_back('"');
        schema += escape_json(fields[i].name);
        schema.push_back('"');
    }
    schema += R"(],"additionalProperties":false})";
    return schema;
}

/**
 * @brief 创建底层 Foxglove RawChannel。
 */
foxglove::RawChannel create_raw_channel(
    std::string_view topic,
    const std::string &schema_data)
{
    foxglove::Schema schema;
    schema.name = std::string(topic);
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte *>(schema_data.data());
    schema.data_len = schema_data.size();

    auto channel_result = foxglove::RawChannel::create(topic, "json", std::move(schema));
    if (!channel_result.has_value())
        throw std::runtime_error(
            std::string("failed to create Foxglove raw channel: ") +
            foxglove::strerror(channel_result.error()));

    return std::move(channel_result.value());
}

/**
 * @brief 向 JSON object 追加浮点值。
 */
void append_json_value(std::string &json, double value)
{
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    json += oss.str();
}

/**
 * @brief 向 JSON object 追加布尔值。
 */
void append_json_value(std::string &json, bool value)
{
    json += value ? "true" : "false";
}

/**
 * @brief 向 JSON object 追加有符号整数值。
 */
void append_json_value(std::string &json, int64_t value)
{
    json += std::to_string(value);
}

/**
 * @brief 向 JSON object 追加无符号整数值。
 */
void append_json_value(std::string &json, uint64_t value)
{
    json += std::to_string(value);
}

} // namespace detail
} // namespace foxglove_viz
