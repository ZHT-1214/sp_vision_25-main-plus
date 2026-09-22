#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#define MAP_KEY(NAME, X, Y, Z) \
    struct NAME##_t { \
        using type = Eigen::Vector3d; \
        static constexpr const char* name = #NAME; \
        static Eigen::Vector3d default_value() { \
            return Eigen::Vector3d(X, Y, Z); \
        } \
    };

namespace awakening::sentry_brain {

template<typename Tuple>
class StaticMap {
public:
    using Point = Eigen::Vector3d;

    struct MapMeta {
        std::string image_path;
        double resolution = 1.0;
        Point origin = Point::Zero();
    };
    StaticMap() {
        register_all();
    }

    static StaticMap& instance() {
        static StaticMap m;
        return m;
    }

    template<typename Key>
    void register_key() {
        declare(Key::name, Key::default_value());
    }

    template<std::size_t... I>
    void register_all_impl(std::index_sequence<I...>) {
        (register_key<std::tuple_element_t<I, Tuple>>(), ...);
    }

    void register_all() {
        register_all_impl(std::make_index_sequence<std::tuple_size_v<Tuple>> {});
    }

    void declare(const std::string& key, const Point& p) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_[key] = p;
    }

    Point get(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return map_.at(key);
    }

    template<typename Key>
    Point get() const {
        return get(Key::name);
    }

    void set(const std::string& key, const Point& p) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_[key] = p;
    }

    template<typename Key>
    void set(const Point& p) {
        set(Key::name, p);
    }

    void load_points_yaml(const std::string& yaml_path) {
        YAML::Node cfg = YAML::LoadFile(yaml_path);
        std::lock_guard<std::mutex> lock(mutex_);

        for (auto it: cfg) {
            std::string key = it.first.as<std::string>();
            if (!map_.count(key)) {
                std::cerr << "Warning: unknown key in YAML: " << key << std::endl;
                continue;
            }

            auto arr = it.second;
            if (!arr.IsSequence() || arr.size() != 3)
                throw std::runtime_error("Point YAML must be [x,y,z]: " + key);

            map_[key] = Point(arr[0].as<double>(), arr[1].as<double>(), arr[2].as<double>());
        }
    }

    void load_ros_map_yaml(const std::string& yaml_path) {
        YAML::Node cfg = YAML::LoadFile(yaml_path);

        meta_.image_path = cfg["image"].as<std::string>();
        meta_.resolution = cfg["resolution"].as<double>();

        auto origin = cfg["origin"];
        meta_.origin =
            Point(origin[0].as<double>(), origin[1].as<double>(), origin[2].as<double>());

        if (!meta_.image_path.empty() && meta_.image_path[0] != '/') {
            const auto pos = yaml_path.find_last_of("/\\");
            if (pos != std::string::npos)
                meta_.image_path = yaml_path.substr(0, pos + 1) + meta_.image_path;
        }

        map_img_ = cv::imread(meta_.image_path, cv::IMREAD_GRAYSCALE);
        if (map_img_.empty())
            throw std::runtime_error("Cannot load map image: " + meta_.image_path);
    }

    Point pixel_to_world(double px, double py) const {
        px /= display_scale_;
        py /= display_scale_;
        py = map_img_.rows - py;

        double x = meta_.origin.x() + px * meta_.resolution;
        double y = meta_.origin.y() + py * meta_.resolution;

        return Point(x, y, 0);
    }

    cv::Point world_to_pixel(const Point& p) const {
        double px = (p.x() - meta_.origin.x()) / meta_.resolution;
        double py = (p.y() - meta_.origin.y()) / meta_.resolution;

        py = map_img_.rows - py;

        return { static_cast<int>(px * display_scale_), static_cast<int>(py * display_scale_) };
    }

    void dump_yaml(const std::string& path) const {
        std::ofstream out(path);

        for (auto& [k, v]: map_) {
            out << k << ": [" << v.x() << ", " << v.y() << ", " << v.z() << "]\n";
        }

        std::cout << "Saved points to: " << path << std::endl;
    }

    void visualize() {
        if (map_img_.empty())
            throw std::runtime_error("Map image not loaded");

        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::resizeWindow("map", 1000, 600);
        cv::setMouseCallback("map", mouseCallback, this);

        std::vector<std::string> keys;
        for (auto& [k, _]: map_)
            keys.push_back(k);

        const int padding = 20;
        const int max_panel_width_limit = 400;
        const int line_h = 20;
        std::vector<cv::Size> text_sizes(keys.size());
        int max_text_width = 0;

        // --- 预计算文字尺寸 ---
        for (size_t i = 0; i < keys.size(); i++) {
            int baseline = 0;
            text_sizes[i] = cv::getTextSize(keys[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            max_text_width = std::max(max_text_width, text_sizes[i].width);
        }

        int panel_width = std::min(max_text_width + padding * 2, max_panel_width_limit);

        while (true) {
            // --- 缩放地图 ---
            cv::Mat map_vis;
            cv::resize(
                map_img_,
                map_vis,
                cv::Size(),
                display_scale_,
                display_scale_,
                cv::INTER_NEAREST
            );
            cv::cvtColor(map_vis, map_vis, cv::COLOR_GRAY2BGR);

            // --- 整体画布 ---
            cv::Mat vis(map_vis.rows, map_vis.cols + panel_width, CV_8UC3, cv::Scalar(40, 40, 40));
            map_vis.copyTo(vis(cv::Rect2f(0, 0, map_vis.cols, map_vis.rows)));

            // --- 绘制地图点 ---
            int idx = 0;
            for (auto& [k, p]: map_) {
                cv::Point pix = world_to_pixel(p);
                cv::Scalar color =
                    (idx == selected_index_) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
                cv::circle(vis, pix, 6, color, -1);
                cv::putText(
                    vis,
                    k,
                    pix + cv::Point(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.4,
                    cv::Scalar(0, 255, 0),
                    1
                );
                idx++;
            }

            // --- 绘制侧边面板 ---
            int start_x = map_vis.cols + padding;
            int start_y = 30;
            for (size_t i = 0; i < keys.size(); i++) {
                std::string text = (i == selected_index_ ? "> " : "  ") + keys[i];
                cv::Size sz = text_sizes[i];
                int x = start_x;
                int y = start_y + i * line_h;

                // 背景框
                cv::rectangle(
                    vis,
                    cv::Point(x - 5, y - sz.height - 2),
                    cv::Point(x + sz.width + 5, y + 4),
                    cv::Scalar(20, 20, 20),
                    -1
                );

                // 文字
                cv::Scalar color =
                    (i == selected_index_) ? cv::Scalar(0, 255, 255) : cv::Scalar(220, 220, 220);
                cv::putText(vis, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }

            cv::imshow("map", vis);

            int key = cv::waitKey(30);

            switch (key) {
                case 'q':
                    return;
                case 's':
                    dump_yaml("points.yaml");
                    break;
                case 81:
                case 2424832: // 左
                    selected_index_ = (selected_index_ - 1 + keys.size()) % keys.size();
                    break;
                case 83:
                case 2555904: // 右
                    selected_index_ = (selected_index_ + 1) % keys.size();
                    break;
            }
        }
    }

private:
    static void mouseCallback(int event, int x, int y, int, void* userdata) {
        if (event != cv::EVENT_LBUTTONDOWN)
            return;

        auto* map = static_cast<StaticMap*>(userdata);
        auto p = map->pixel_to_world(x, y);

        int idx = 0;
        for (auto& [k, v]: map->map_) {
            if (idx == map->selected_index_) {
                v = p;
                std::cout << "Set " << k << " -> " << p.x() << ", " << p.y() << std::endl;
                break;
            }
            idx++;
        }
    }

private:
    MapMeta meta_;
    cv::Mat map_img_;
    double display_scale_ = 3.0;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, Point> map_;
    int selected_index_ = 0;
};

} // namespace awakening::sentry_brain