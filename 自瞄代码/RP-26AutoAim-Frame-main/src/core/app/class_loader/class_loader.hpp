#pragma once

#include <concepts>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "PluginBase.hpp"

namespace app
{

class PluginBase;

class PluginFactoryBase
{
public:
    virtual std::shared_ptr<PluginBase> create() = 0;
    const std::string& name() const { return m_plugin_name; }

    PluginFactoryBase(const std::string &plugin_name) : m_plugin_name(plugin_name) {}
    virtual ~PluginFactoryBase() = default;

protected:
    std::string m_plugin_name;
};

template <typename T>
concept Plugin =
    !std::is_reference_v<T> &&
    !std::is_pointer_v<T> &&
    !std::is_const_v<T> &&
    std::derived_from<T, PluginBase>;

template <Plugin T>
class PluginFactory : public PluginFactoryBase
{
public:
    PluginFactory(const std::string &plugin_name) : PluginFactoryBase(plugin_name) {}

    std::shared_ptr<PluginBase> create() override
    {
        return std::make_shared<T>();
    }
};

void load_plugin_library(const std::filesystem::path &library_path);
bool add_plugin_factory(std::unique_ptr<PluginFactoryBase> plugin);
bool exists(const std::string &name);
bool remove_plugin_factory(const std::string &name);
std::shared_ptr<PluginBase> create_plugin(const std::string &name);

} // namespace app

#define REGISTER_PLUGIN(NAME, TYPE) \
    namespace \
    { \
        struct PluginRegister \
        {\
            PluginRegister() \
            {\
                bool ret = app::add_plugin_factory(std::make_unique<app::PluginFactory<TYPE>>(NAME)); \
                if (!ret) \
                    throw std::runtime_error(std::string("Failed to load plugin named") + NAME); \
            }\
        };\
        \
        static PluginRegister reg; \
    }
