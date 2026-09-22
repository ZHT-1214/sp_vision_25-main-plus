#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/core.hpp>

/// @brief 进程级图像可视化管理器。
///
/// 实现位于 libimg_viz_lib.so，所有主程序和插件都会链接同一份 instance()，从而共享唯一的
/// HighGUI 线程。构造实例或设置开关不会启动线程；首次提交图像或 GUI 任务时才会启动。
class ImgViz
{
public:
    static ImgViz &instance();

    ImgViz(const ImgViz &) = delete;
    ImgViz &operator=(const ImgViz &) = delete;

    /// @brief 初始化可视化管理器。只能在启动时调用一次。
    static void init(bool enabled);

    [[nodiscard]] static bool enabled();

    /// @brief 深拷贝图像并加入待显示队列。
    static void enqueue_image_copy(std::string_view window_name, const cv::Mat &image);

    /// @brief 不复制像素数据地加入待显示队列。
    /// 调用方必须保证底层像素内存在显示完成前有效，且不被并发修改或复用。
    static void enqueue_image_zero_copy(std::string_view window_name, const cv::Mat &image);

private:
    ImgViz() = default;
    ~ImgViz();

    // 用于可视化线程缓启动
    void ensure_worker();
    void visualize();

    enum class State
    {
        Uninitialized,
        Enabled,
        Disabled,
        Stopping
    };

    struct WindowState
    {
        cv::Mat image;
        bool is_dirty = false;
    };

    std::atomic<State> m_state{State::Uninitialized};
    mutable std::mutex m_mutex;
    std::unordered_map<std::string, WindowState> m_windows;
    std::unique_ptr<std::thread> m_worker;
};
