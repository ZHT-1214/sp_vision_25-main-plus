#include "img_viz.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>

ImgViz &ImgViz::instance()
{
    static ImgViz instance;
    return instance;
}

ImgViz::~ImgViz()
{
    State expected = State::Enabled;
    m_state.compare_exchange_strong(expected, State::Stopping);

    if (m_worker && m_worker->joinable())
        m_worker->join();
}

void ImgViz::init(bool enabled)
{
    auto &self = instance();
    State expected = State::Uninitialized;
    if (!self.m_state.compare_exchange_strong(expected, enabled ? State::Enabled : State::Disabled))
        throw std::logic_error("img_viz::init() can only be called once");
}

bool ImgViz::enabled()
{
    return instance().m_state.load(std::memory_order_relaxed) == State::Enabled;
}

void ImgViz::enqueue_image_copy(std::string_view window_name, const cv::Mat &image)
{
    if (image.empty())
        return;

    enqueue_image_zero_copy(window_name, image.clone());
}

void ImgViz::enqueue_image_zero_copy(std::string_view window_name, const cv::Mat &image)
{
    auto &self = instance();
    if (image.empty() || self.m_state.load(std::memory_order_relaxed) != State::Enabled)
        return;

    {
        std::lock_guard lock(self.m_mutex);
        auto &window = self.m_windows[std::string(window_name)];
        window.image = image;
        window.is_dirty = true;
    }
    self.ensure_worker();
}

void ImgViz::ensure_worker()
{
    if (m_state.load(std::memory_order_relaxed) != State::Enabled)
        return;

    std::lock_guard lock(m_mutex);
    if (!m_worker)
        m_worker = std::make_unique<std::thread>(&ImgViz::visualize, this);
}

void ImgViz::visualize()
{
    while (m_state.load(std::memory_order_relaxed) != State::Stopping)
    {
        std::vector<std::pair<std::string, cv::Mat>> to_draw;
        {
            std::lock_guard lock(m_mutex);
            for (auto &[name, window] : m_windows)
            {
                if (window.is_dirty)
                {
                    to_draw.emplace_back(name, std::move(window.image));
                    window.is_dirty = false;
                }
            }
        }

        for (const auto &[name, image] : to_draw)
        {
            if (!image.empty())
            {
                cv::namedWindow(name, cv::WINDOW_NORMAL);
                cv::imshow(name, image);
            }
        }

        cv::waitKeyEx(1);
    }
}
