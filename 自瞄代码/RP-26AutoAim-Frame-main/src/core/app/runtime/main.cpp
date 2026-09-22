#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core/persistence.hpp>

#include "class_loader.hpp"
#include "context.hpp"
#include "google_logger/google_logger.hpp"
#include "img_viz.hpp"


struct PluginConfig
{
    std::string name;
    std::string library;
    std::string attach_to;
};

using PluginThreadGroup = std::vector<std::shared_ptr<app::PluginBase>>;

struct RuntimeConfig
{
    std::vector<PluginConfig> plugin_configs;
    bool visualize_enabled = true;
};

RuntimeConfig read_runtime_config(const std::filesystem::path &config_path)
{
    std::string config_path_string = config_path.string();
    cv::FileStorage config(config_path_string, cv::FileStorage::READ);

    if (!config.isOpened())
    {
        std::ostringstream stream;
        stream << "failed to open config: " << config_path;
        throw std::runtime_error(stream.str());
    }

    RuntimeConfig runtime_config;

    const cv::FileNode visualize_node = config["visualize"];
    if (!visualize_node.empty())
    {
        int visualize_enabled = 1;
        visualize_node >> visualize_enabled;
        runtime_config.visualize_enabled = visualize_enabled != 0;
    }

    std::string list_key = "plugins";
    cv::FileNode list_key_node = config["loading_plugin_list"];
    if (!list_key_node.empty() && list_key_node.isString())
    {
        list_key_node >> list_key;
    }

    cv::FileNode plugins_node = config[list_key];
    if (plugins_node.empty() || !plugins_node.isSeq())
    {
        std::ostringstream stream;
        stream << "config must contain a sequence named '" << list_key << "': " << config_path;
        throw std::runtime_error(stream.str());
    }

    for (const auto &plugin_node : plugins_node)
    {
        if (!plugin_node.isMap())
            throw std::runtime_error("each plugin config item must be an object");

        PluginConfig plugin_config;
        plugin_node["name"] >> plugin_config.name;
        plugin_node["library"] >> plugin_config.library;
        cv::FileNode attach_to_node = plugin_node["AttachTo"];
        if (!attach_to_node.empty())
            attach_to_node >> plugin_config.attach_to;

        if (plugin_config.name.empty() || plugin_config.library.empty())
            throw std::runtime_error("plugin config item must contain name and library");

        runtime_config.plugin_configs.emplace_back(std::move(plugin_config));
    }

    return runtime_config;
}

std::vector<PluginThreadGroup> load_and_group_plugins(
    const std::vector<PluginConfig> &plugin_configs,
    const std::filesystem::path &library_dir,
    app::Context &context)
{
    std::unordered_map<std::string, std::size_t> name2group_map;
    std::vector<PluginThreadGroup> threadgroups;

    for (const PluginConfig &plugin_config : plugin_configs)
    {
        if (name2group_map.contains(plugin_config.name))
        {
            std::ostringstream stream;
            stream << "duplicate plugin config name: " << plugin_config.name;
            throw std::runtime_error(stream.str());
        }


        app::load_plugin_library(library_dir / plugin_config.library);
        std::shared_ptr<app::PluginBase> plugin = app::create_plugin(plugin_config.name);
        if (plugin == nullptr)
        {
            std::ostringstream stream;
            stream << "plugin is not registered after loading library: " << plugin_config.name;
            throw std::runtime_error(stream.str());
        }
        else
            context.add_plugin(plugin);


        if (plugin_config.attach_to.empty())
        {
            threadgroups.push_back({ plugin });
            name2group_map[plugin_config.name] = threadgroups.size() - 1;
        }
        else
        {
            auto target_group_it = name2group_map.find(plugin_config.attach_to);
            if (target_group_it == name2group_map.end())
            {
                std::ostringstream stream;
                stream << "The AttachTo target must be a previous plugin: plugin=" << plugin_config.name
                       << ", AttachTo=" << plugin_config.attach_to;
                throw std::runtime_error(stream.str());
            }

            size_t group_index = target_group_it->second;
            threadgroups[group_index].emplace_back(plugin);
            name2group_map[plugin_config.name] = group_index;
        }
    }

    return threadgroups;
}

void run_plugin_thread_groups(
    const std::vector<PluginThreadGroup> &plugin_thread_groups,
    const app::Context &context)
{
    std::vector<std::thread> plugin_threads;
    plugin_threads.reserve(plugin_thread_groups.size());

    for (const PluginThreadGroup &one_group : plugin_thread_groups)
    {
        plugin_threads.emplace_back(
            [plugins = one_group, &context]
            {
                while (true)
                {
                    for (const std::shared_ptr<app::PluginBase> &plugin : plugins)
                        plugin->process(context);
                }
            }
        );
    }

    for (std::thread &plugin_thread : plugin_threads)
        plugin_thread.join();
}

int main(int argc, char const *argv[])
{
    google_log_config::init_google_log(nullptr);
    std::atexit(google::ShutdownGoogleLogging);

    const std::filesystem::path executable_path = std::filesystem::canonical("/proc/self/exe");
    const std::filesystem::path executable_dir = executable_path.parent_path();
    const std::filesystem::path config_path = executable_dir.parent_path() / "config" / "runtime_config.json";
    
    const RuntimeConfig runtime_config = read_runtime_config(config_path);
    ImgViz::init(runtime_config.visualize_enabled);

    app::Context context;
    const std::vector<PluginThreadGroup> plugin_thread_groups =
        load_and_group_plugins(runtime_config.plugin_configs, executable_dir, context);

    context.check_io_network();
    std::cout << "context构建完成" << std::endl;
    LOG(INFO) << "[app] context initialized; plugin thread groups are running";

    run_plugin_thread_groups(plugin_thread_groups, context);

    return 0;
}
