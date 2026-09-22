#pragma once
#include "utils/tf.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace awakening::utils::tf {

struct LinkBuffer {
    TimePoseBuffer buffer;
    LinkBuffer(size_t size = 4096): buffer(size) {}
};

// ---------------- Edge ----------------
template<typename FrameEnum>
struct Edge {
    FrameEnum parent;
    FrameEnum child;
};

// ---------------- RobotTF ----------------
template<typename FrameEnum, size_t N, bool Static>
class RobotTF {
public:
    using Ptr = std::shared_ptr<RobotTF>;
    static Ptr create() {
        return std::make_shared<RobotTF>();
    }

    void add_edge(FrameEnum parent, FrameEnum child) {
        std::lock_guard<std::mutex> lock(path_cache_mutex_);
        if (!valid_frame(parent) || !valid_frame(child)) {
            if constexpr (Static) {
                throw std::out_of_range(
                    "Invalid TF edge " + frame_to_string(parent) + " -> " + frame_to_string(child)
                );
            } else {
                return;
            }
        }
        if (has_direct_edge(parent, child))
            return;

        edges_.push_back({ parent, child });
        adjacency_[to_index(parent)].push_back(child);
        adjacency_[to_index(child)].push_back(parent);
        directed_edge_[to_index(parent)][to_index(child)] = true;
        path_cache_ = {};
    }

    bool push(FrameEnum parent, FrameEnum child, const TimePoint& t, const ISO3& pose_in_parent) {
        if (!valid_frame(parent) || !valid_frame(child)) {
            if constexpr (Static) {
                throw std::out_of_range(
                    "Invalid TF edge " + frame_to_string(parent) + " -> " + frame_to_string(child)
                );
            } else {
                return false;
            }
        }

        if (!has_direct_edge(parent, child)) {
            if constexpr (Static) {
                throw std::runtime_error(
                    "Attempt to push pose for undefined edge " + frame_to_string(parent) + " -> "
                    + frame_to_string(child)
                );
            } else {
                return false;
            }
        }

        buffers_[to_index(parent)][to_index(child)].buffer.push(t, pose_in_parent);
        return true;
    }

    ISO3 pose_a_in_b(FrameEnum a, FrameEnum b, const TimePoint& t) const {
        if (!valid_frame(a) || !valid_frame(b)) {
            if constexpr (Static) {
                throw std::out_of_range(
                    "Invalid TF query " + frame_to_string(a) + " in " + frame_to_string(b)
                );
            } else {
                return ISO3::Identity();
            }
        }

        auto path = find_path(b, a);
        if (!path) {
            if constexpr (Static) {
                throw std::runtime_error(
                    "No path from " + frame_to_string(a) + " to " + frame_to_string(b)
                );
            } else {
                std::cout << "No path from " + frame_to_string(a) + " to " + frame_to_string(b)
                          << std::endl;
                return ISO3::Identity();
            }
        }

        ISO3 T = ISO3::Identity();
        for (size_t i = 0; i + 1 < path->size(); ++i) {
            auto A = (*path)[i];
            auto B = (*path)[i + 1];

            ISO3 t_ab = ISO3::Identity();
            if (has_direct_edge(A, B)) {
                t_ab = buffers_[to_index(A)][to_index(B)].buffer.get(t);
            } else if (has_direct_edge(B, A)) {
                t_ab = buffers_[to_index(B)][to_index(A)].buffer.get(t).inverse();
            } else {
                if constexpr (Static) {
                    throw std::runtime_error(
                        "Broken path edge between " + frame_to_string(A) + " and "
                        + frame_to_string(B)
                    );
                } else {
                    t_ab = ISO3::Identity();
                }
            }

            T = T * t_ab;
        }
        return T;
    }

    std::vector<Edge<FrameEnum>> get_edges() const {
        return edges_;
    }

private:
    std::array<std::array<LinkBuffer, N>, N> buffers_;
    std::vector<Edge<FrameEnum>> edges_;
    std::array<std::vector<FrameEnum>, N> adjacency_;
    std::array<std::array<bool, N>, N> directed_edge_ {};
    mutable std::array<std::array<std::optional<std::vector<FrameEnum>>, N>, N> path_cache_ {};
    mutable std::mutex path_cache_mutex_;

    bool has_direct_edge(FrameEnum A, FrameEnum B) const {
        return valid_frame(A) && valid_frame(B) && directed_edge_[to_index(A)][to_index(B)];
    }

    static size_t to_index(FrameEnum frame) {
        using Underlying = std::underlying_type_t<FrameEnum>;
        return static_cast<size_t>(static_cast<Underlying>(frame));
    }

    static bool valid_frame(FrameEnum frame) {
        using Underlying = std::underlying_type_t<FrameEnum>;
        auto value = static_cast<Underlying>(frame);
        if constexpr (std::is_signed_v<Underlying>) {
            if (value < 0)
                return false;
        }
        return static_cast<size_t>(value) < N;
    }

    static std::string frame_to_string(FrameEnum frame) {
        using Underlying = std::underlying_type_t<FrameEnum>;
        return std::to_string(static_cast<Underlying>(frame));
    }

    std::optional<std::vector<FrameEnum>> find_path(FrameEnum start, FrameEnum goal) const {
        std::lock_guard<std::mutex> lock(path_cache_mutex_);
        auto& cached = path_cache_[to_index(start)][to_index(goal)];
        if (cached)
            return cached;

        std::array<bool, N> visited {};
        std::array<std::optional<FrameEnum>, N> parent {};
        std::queue<FrameEnum> q;

        q.push(start);
        visited[to_index(start)] = true;

        while (!q.empty()) {
            auto node = q.front();
            q.pop();
            if (node == goal)
                break;

            for (auto neighbor: adjacency_[to_index(node)]) {
                auto neighbor_index = to_index(neighbor);
                if (visited[neighbor_index])
                    continue;
                visited[neighbor_index] = true;
                parent[neighbor_index] = node;
                q.push(neighbor);
            }
        }

        if (!visited[to_index(goal)])
            return std::nullopt;

        std::vector<FrameEnum> path;
        for (auto node = goal;; node = *parent[to_index(node)]) {
            path.push_back(node);
            if (node == start)
                break;
        }
        std::reverse(path.begin(), path.end());
        cached = std::move(path);
        return cached;
    }
};

} // namespace awakening::utils::tf
