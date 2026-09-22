#pragma once

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "PluginBase.hpp"
#include "latest_buffer/latest_buffer.hpp"
#include "latest_channel/latest_channel.hpp"

namespace app
{

enum class CommunicationToolType
{
    LatestBuffer,
    LatestChannel
};

enum class Direction
{
    Input,
    Output
};

class Context
{

public:
    Context() = default;

    Context(const Context &) = delete;
    Context &operator=(const Context &) = delete;

    void add_plugin(std::shared_ptr<PluginBase> plugin)
    {
        if (plugin == nullptr)
            throw std::runtime_error("[Context] cannot add null plugin");

        auto same_plugin_it = std::ranges::find_if(
            m_plugins,
            [&plugin](const auto &registered_plugin){
                return registered_plugin.get() == plugin.get();
            });

        if (same_plugin_it != m_plugins.end())
            throw std::runtime_error("[Context] duplicate plugin node");

        m_plugins.emplace_back(std::move(plugin));
    }

    void check_io_network()
    {
        clear_network();

        for (const auto &plugin : m_plugins)
            plugin->declare(*this);

        std::vector<std::vector<const IODeclaration *>> declaration_groups;

        // 检查是否有不相容的声明，如果都相容则把相同的声明归为一组存到 declaration_groups 之中
        for (const auto &declaration : m_declarations)
        {
            bool is_grouped = false;
            for (auto &declaration_group : declaration_groups)
            {
                const auto &representative_declaration = *declaration_group.front();
                if (!representative_declaration.is_compatible_with(declaration))
                    throw_declaration_incompatible_error(
                        "declarations are incompatible",
                        representative_declaration,
                        declaration);

                if (!representative_declaration.is_same_as(declaration))
                    continue;

                declaration_group.emplace_back(&declaration);
                is_grouped = true;
                break;
            }

            // 遍历完了仍未分组的，新开一组
            if (!is_grouped)
                declaration_groups.push_back({&declaration});
        }

        // 对 declaration_groups 中的每一组去重，防止一个插件重复声明
        for (auto &declarations : declaration_groups)
        {
            std::vector<const IODeclaration *> deduped_declarations;

            for (const auto *declaration : declarations)
            {
                auto same_plugin_declaration = std::ranges::find_if(
                    deduped_declarations,
                    [&](const auto *candidate){
                        return candidate->plugin == declaration->plugin;
                    });

                // 按模块去重后的声明列表里面不存在当前声明，则加入声明列表
                if (same_plugin_declaration == deduped_declarations.end())
                    deduped_declarations.emplace_back(declaration);
                // 若存在且方向不同
                else if ((*same_plugin_declaration)->direction != declaration->direction)
                    throw_declaration_incompatible_error(
                        "same plugin cannot use the same communication tool with different directions",
                        **same_plugin_declaration,
                        *declaration);
            }

            declarations = std::move(deduped_declarations);
        }

        // 保证 LatestBuffer 至多一个消费者，LatestChannel 至多一个生产者
        for (const auto &declarations : declaration_groups)
        {
            const auto communication_tool_type = declarations.front()->communication_tool_type;

            if (communication_tool_type == CommunicationToolType::LatestBuffer)
            {
                const IODeclaration *first_input_declaration = nullptr;
                for (const auto *declaration : declarations)
                {
                    if (declaration->direction != Direction::Input)
                        continue;

                    if (first_input_declaration != nullptr)
                        throw_declaration_incompatible_error(
                            "LatestBuffer can have at most one consumer",
                            *first_input_declaration,
                            *declaration);

                    first_input_declaration = declaration;
                }
            }
            else
            {
                const IODeclaration *first_output_declaration = nullptr;
                for (const auto *declaration : declarations)
                {
                    if (declaration->direction != Direction::Output)
                        continue;

                    if (first_output_declaration != nullptr)
                        throw_declaration_incompatible_error(
                            "LatestChannel can have at most one producer",
                            *first_output_declaration,
                            *declaration);

                    first_output_declaration = declaration;
                }
            }
        }

        // 若某个声明没有上游或者没有下游则发出警告
        for (const auto &declarations : declaration_groups)
        {
            const IODeclaration &representative_declaration = *declarations.front();
            const bool has_output = std::ranges::any_of(
                declarations,
                [](const auto *declaration){
                    return declaration->direction == Direction::Output;
                });
            const bool has_input = std::ranges::any_of(
                declarations,
                [](const auto *declaration){
                    return declaration->direction == Direction::Input;
                });

            if (!has_output || !has_input)
                std::cerr << "[Context] warning: This communication tool has no "
                          << (!has_output ? "publisher" : "subscriber")
                          << ": communication_tool_type=" << (representative_declaration.communication_tool_type == CommunicationToolType::LatestBuffer ? "LatestBuffer" : "LatestChannel")
                          << ", data_type=" << representative_declaration.data_type_index.name()
                          << ", topic='" << representative_declaration.topic << "'"
                          << std::endl;
        }

        // 每个声明组都创建通信工具，没有发布者或订阅者只在前面输出 warning
        for (const auto &declarations : declaration_groups)
        {
            auto communication_tool = declarations.front()->communication_tool_factory();

            for (const auto *declaration : declarations)
                m_communication_infos[declaration->plugin].emplace_back(*declaration,
                                                                         communication_tool);
        }
    }

    template <typename T>
    void declare_output_buffer(const PluginBase *plugin, std::string topic = {})
    {
        declare_buffer<T>(std::move(topic), Direction::Output, plugin);
    }

    template <typename T>
    void declare_input_buffer(const PluginBase *plugin, std::string topic = {})
    {
        declare_buffer<T>(std::move(topic), Direction::Input, plugin);
    }

    template <typename T>
    void declare_output_channel(const PluginBase *plugin, std::string topic = {})
    {
        declare_channel<T>(std::move(topic), Direction::Output, plugin);
    }

    template <typename T>
    void declare_input_channel(const PluginBase *plugin, std::string topic = {})
    {
        declare_channel<T>(std::move(topic), Direction::Input, plugin);
    }

    template <typename T>
    typename LatestBuffer<T>::Publisher get_buffer_publisher(const PluginBase *plugin, std::string topic = {}) const
    {
        auto *communication_tool = find_communication_tool_for<T>(topic, CommunicationToolType::LatestBuffer, Direction::Output, plugin);
        if (communication_tool == nullptr)
            throw std::runtime_error("[Context] buffer publisher is not registered for plugin");

        auto *typed_buffer = dynamic_cast<LatestBuffer<T> *>(communication_tool);
        if (typed_buffer == nullptr)
            throw std::runtime_error("[Context] buffer publisher type mismatch");

        return typename LatestBuffer<T>::Publisher(*typed_buffer);
    }

    template <typename T>
    typename LatestBuffer<T>::Subscriber get_buffer_subscriber(const PluginBase *plugin, std::string topic = {}) const
    {
        auto *communication_tool = find_communication_tool_for<T>(topic, CommunicationToolType::LatestBuffer, Direction::Input, plugin);
        if (communication_tool == nullptr)
            throw std::runtime_error("[Context] buffer subscriber is not registered for plugin");

        auto *typed_buffer = dynamic_cast<LatestBuffer<T> *>(communication_tool);
        if (typed_buffer == nullptr)
            throw std::runtime_error("[Context] buffer subscriber type mismatch");

        return typename LatestBuffer<T>::Subscriber(*typed_buffer);
    }

    template <typename T>
    typename LatestChannel<T>::Publisher get_channel_publisher(const PluginBase *plugin, std::string topic = {}) const
    {
        auto *communication_tool = find_communication_tool_for<T>(topic, CommunicationToolType::LatestChannel, Direction::Output, plugin);
        if (communication_tool == nullptr)
            throw std::runtime_error("[Context] channel publisher is not registered for plugin");

        auto *typed_channel = dynamic_cast<LatestChannel<T> *>(communication_tool);
        if (typed_channel == nullptr)
            throw std::runtime_error("[Context] channel publisher type mismatch");

        return typename LatestChannel<T>::Publisher(*typed_channel);
    }

    template <typename T>
    typename LatestChannel<T>::Subscriber get_channel_subscriber(const PluginBase *plugin,
                                                                 std::string topic = {},
                                                                 bool is_receive_existing_msg = false) const
    {
        auto *communication_tool = find_communication_tool_for<T>(topic, CommunicationToolType::LatestChannel, Direction::Input, plugin);
        if (communication_tool == nullptr)
            throw std::runtime_error("[Context] channel subscriber is not registered for plugin");

        auto *typed_channel = dynamic_cast<LatestChannel<T> *>(communication_tool);
        if (typed_channel == nullptr)
            throw std::runtime_error("[Context] channel subscriber type mismatch");

        return typename LatestChannel<T>::Subscriber(*typed_channel, is_receive_existing_msg);
    }

private:
    using CommunicationToolFactory = std::function<std::shared_ptr<CommunicationToolBase>()>;

    struct IODeclaration
    {
        std::string topic;
        std::type_index data_type_index;
        CommunicationToolType communication_tool_type;
        Direction direction;
        const PluginBase *plugin;
        CommunicationToolFactory communication_tool_factory;

        bool is_compatible_with(const IODeclaration &other) const
        {
            if (!topic.empty() && !other.topic.empty())
            {
                if (topic != other.topic)
                    return true;

                return data_type_index == other.data_type_index &&
                       communication_tool_type == other.communication_tool_type;
            }

            if (topic.empty() && other.topic.empty())
            {
                if (data_type_index != other.data_type_index)
                    return true;

                return communication_tool_type == other.communication_tool_type;
            }

            return true;
        }

        bool is_same_as(const IODeclaration &other) const
        {
            return topic == other.topic &&
                   data_type_index == other.data_type_index &&
                   communication_tool_type == other.communication_tool_type;
        }
    };

    std::vector<std::shared_ptr<PluginBase>> m_plugins;
    std::vector<IODeclaration> m_declarations;
    std::unordered_map<const PluginBase *, std::vector<std::tuple<IODeclaration, std::shared_ptr<CommunicationToolBase>>>> m_communication_infos;

private:
    [[noreturn]] static void throw_declaration_incompatible_error(const char *message,
                                                                  const IODeclaration &declaration1,
                                                                  const IODeclaration &declaration2)
    {
        std::ostringstream stream;
        stream << "[Context] " << message
               << ": declaration1={topic='" << declaration1.topic << "'"
               << ", data_type=" << declaration1.data_type_index.name()
               << ", communication_tool_type=" << (declaration1.communication_tool_type == CommunicationToolType::LatestBuffer ? "LatestBuffer" : "LatestChannel")
               << ", direction=" << (declaration1.direction == Direction::Input ? "Input" : "Output")
               << ", plugin=" << static_cast<const void *>(declaration1.plugin) << "}"
               << ", declaration2={topic='" << declaration2.topic << "'"
               << ", data_type=" << declaration2.data_type_index.name()
               << ", communication_tool_type=" << (declaration2.communication_tool_type == CommunicationToolType::LatestBuffer ? "LatestBuffer" : "LatestChannel")
               << ", direction=" << (declaration2.direction == Direction::Input ? "Input" : "Output")
               << ", plugin=" << static_cast<const void *>(declaration2.plugin) << "}";

        throw std::runtime_error(stream.str());
    }

    template <typename T>
    void declare_buffer(std::string topic,
                        Direction direction,
                        const PluginBase *plugin)
    {
        declare_port(std::move(topic),
                     std::type_index(typeid(T)),
                     CommunicationToolType::LatestBuffer,
                     direction,
                     plugin,
                     [](){ return std::make_shared<LatestBuffer<T>>(); });
    }

    template <typename T>
    void declare_channel(std::string topic,
                         Direction direction,
                         const PluginBase *plugin)
    {
        declare_port(std::move(topic),
                     std::type_index(typeid(T)),
                     CommunicationToolType::LatestChannel,
                     direction,
                     plugin,
                     [](){ return std::make_shared<LatestChannel<T>>(); });
    }

    void declare_port(std::string topic,
                      std::type_index data_type_index,
                      CommunicationToolType communication_tool_type,
                      Direction direction,
                      const PluginBase *plugin,
                      CommunicationToolFactory communication_tool_factory)
    {
        if (plugin == nullptr)
            throw std::runtime_error("[Context] cannot declare io for null plugin");

        auto plugin_it = std::ranges::find_if(
            m_plugins,
            [plugin](const auto &registered_plugin){
                return registered_plugin.get() == plugin;
            });

        if (plugin_it == m_plugins.end())
            throw std::runtime_error("[Context] io declaration uses an unknown plugin node");

        m_declarations.push_back({std::move(topic),
                                  data_type_index,
                                  communication_tool_type,
                                  direction,
                                  plugin,
                                  std::move(communication_tool_factory)});
    }

    void clear_network()
    {
        m_declarations.clear();
        m_communication_infos.clear();
    }

    template <typename T>
    CommunicationToolBase *find_communication_tool_for(const std::string &topic,
                                                       CommunicationToolType communication_tool_type,
                                                       Direction direction,
                                                       const PluginBase *plugin) const
    {
        if (plugin == nullptr)
            throw std::runtime_error("[Context] cannot get communication tool for null plugin");

        auto plugin_it = m_communication_infos.find(plugin);
        if (plugin_it == m_communication_infos.end())
            return nullptr;

        const auto data_type_index = std::type_index(typeid(T));
        for (const auto &[declaration, communication_tool] : plugin_it->second)
            if (declaration.communication_tool_type == communication_tool_type &&
                declaration.data_type_index == data_type_index &&
                declaration.topic == topic &&
                declaration.direction == direction)
                return communication_tool.get();

        return nullptr;
    }
};

}// namespace app
