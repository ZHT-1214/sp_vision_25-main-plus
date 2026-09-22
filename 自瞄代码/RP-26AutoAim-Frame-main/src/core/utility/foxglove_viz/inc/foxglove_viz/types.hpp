#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace foxglove_viz
{

/// @brief 所有 publisher 共享的基础元信息。
struct PublisherMetaInfo
{
    /// @brief Publisher 的用途类型。
    enum class PublisherKind
    {
        Data,
        Scene,
    };

    std::string topic;
    PublisherKind kind;
};

namespace detail
{

/// @brief 针对字段类型“标量”的约束
template <typename T>
concept FieldBasicType =
    std::same_as<T, double> ||
    std::same_as<T, bool> ||
    std::same_as<T, int64_t> ||
    std::same_as<T, uint64_t>;

template <typename T>
struct IsMsgVector : std::false_type {};

template <typename T, typename Allocator>
struct IsMsgVector<std::vector<T, Allocator>> : std::bool_constant<FieldBasicType<T>> {};

template <typename T>
struct IsMsgArray : std::false_type {};

template <typename T, std::size_t N>
struct IsMsgArray<std::array<T, N>> : std::bool_constant<FieldBasicType<T>> {};

} // namespace detail

/// @brief 针对字段类型的约束
template <typename T>
concept FieldType =
    detail::FieldBasicType<T> ||
    detail::IsMsgVector<T>::value ||
    detail::IsMsgArray<T>::value;

} // namespace foxglove_viz
