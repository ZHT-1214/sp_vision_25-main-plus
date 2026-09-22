#pragma once

#include <filesystem>
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace google_log_config
{
/**
 * @brief 初始化 Google glog（日志目录、输出策略与滚动配置）。
 * @param program_name 进程名，用于日志文件前缀；为 nullptr 时使用 "Infantry_2026"。
 */
inline void init_google_log(const char *program_name)
{
    google::InitGoogleLogging(program_name != nullptr ? program_name : "Infantry_2026");
    google::InstallFailureSignalHandler();

    std::filesystem::path log_dir = "../logs";

    std::error_code create_dir_ec;
    std::filesystem::create_directories(log_dir, create_dir_ec);
    if (create_dir_ec)
    {
        std::cerr << "[glog] failed to create log dir: " << log_dir
                  << ", error: " << create_dir_ec.message() << std::endl;
    }

    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = 0;

    FLAGS_log_prefix = true;

    FLAGS_logbuflevel = 0;
    FLAGS_logbufsecs = 60;
    FLAGS_minloglevel = 0;

    FLAGS_log_dir = log_dir.string();
    FLAGS_logfile_mode = 0644;
    FLAGS_log_link = "";

    FLAGS_v = 0;

    FLAGS_max_log_size = 1800;
    FLAGS_stop_logging_if_full_disk = false;
}
} // namespace google_log_config
