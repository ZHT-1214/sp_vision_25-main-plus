#include "class_loader.hpp"

#include <dlfcn.h>
#include <filesystem>
#include <map>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace app
{

static auto& plugin_library_handles()
{
    static std::map<std::filesystem::path, void *> handles;
    return handles;
}

static auto& plugin_factories()
{
    static std::vector<std::unique_ptr<PluginFactoryBase>> factories;
    return factories;
}

void load_plugin_library(const std::filesystem::path &library_path)
{
    const auto normalized_path = std::filesystem::absolute(library_path).lexically_normal();
    auto &handles = plugin_library_handles();

    if (handles.contains(normalized_path))
        return;

    dlerror();
    void *handle = dlopen(normalized_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (handle == nullptr)
    {
        const char *error_message = dlerror();
        std::ostringstream stream;
        stream << "[class_loader] failed to load library: " << normalized_path;
        if (error_message != nullptr)
            stream << ": " << error_message;
        throw std::runtime_error(stream.str());
    }

    handles.emplace(normalized_path, handle);
}

bool add_plugin_factory(std::unique_ptr<PluginFactoryBase> plugin)
{
    auto &factories = plugin_factories();
    auto same_name_it = std::ranges::find_if(
        factories,
        [&plugin](const std::unique_ptr<PluginFactoryBase> &obj){
            if (obj->name() == plugin->name())
                return true;
            else
                return false;
        }
    );

    if (same_name_it != factories.end())
        return false;

    factories.emplace_back(std::move(plugin));
    return true;
}

bool remove_plugin_factory(const std::string &name)
{
    auto &factories = plugin_factories();
    auto same_name_it = std::ranges::find_if(
        factories,
        [&name](const std::unique_ptr<PluginFactoryBase> &obj){
            if (obj->name() == name)
                return true;
            else
                return false;
        }
    );

    if (same_name_it == factories.end())
        return false;

    factories.erase(same_name_it);
    return true;
}

bool exists(const std::string &name)
{
    auto &factories = plugin_factories();
    auto same_name_it = std::ranges::find_if(
        factories,
        [&name](const std::unique_ptr<PluginFactoryBase> &obj){
            if (obj->name() == name)
                return true;
            else
                return false;
        }
    );

    return same_name_it != factories.end();
}

std::shared_ptr<PluginBase> create_plugin(const std::string &name)
{
    auto &factories = plugin_factories();
    auto same_name_it = std::ranges::find_if(
        factories,
        [&name](const std::unique_ptr<PluginFactoryBase> &obj){
            if (obj->name() == name)
                return true;
            else
                return false;
        }
    );

    if (same_name_it == factories.end())
        return nullptr;
    else
        return (**same_name_it).create();
}

} // namespace app
