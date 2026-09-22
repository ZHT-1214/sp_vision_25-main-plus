#pragma once
#include "node.hpp"
#include "utils/logger.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <tbb/task_group.h>

namespace awakening {

class Scheduler {
public:
    using clock = std::chrono::steady_clock;
    inline static unsigned int __hardware_concurrency = std::thread::hardware_concurrency();

    explicit Scheduler(size_t threads = __hardware_concurrency * 2.0):
        worker_count(threads ? threads : 1) {}

    ~Scheduler() {
        stop();
    }

    template<typename InputPair, typename... OutputPairs, typename Fn>
    void register_task(std::string n, Fn&& fn) {
        auto node = TaskNode<InputPair, OutputPairs...>::create(std::forward<Fn>(fn));
        node->name = std::move(n);

        auto inputs = node->input_tags();
        if (inputs.empty()) {
            throw std::runtime_error("Task must have input");
        }

        static_tasks_snapshot_[inputs.front()].push_back(node);
        built_ = false;
    }

    template<typename... OutputPairs>
    [[nodiscard]] size_t register_source(std::string n) {
        auto node = SourceNode<OutputPairs...>::create();
        node->name = std::move(n);

        source_snapshot_.push_back(node);
        built_ = false;
        return source_snapshot_.size() - 1;
    }

    template<typename... OutputPairs, typename Fn>
    void runtime_push_source(size_t snap_id, Fn&& fn) {
        if (!built_)
            build();

        if (!is_running())
            return;

        if (snap_id >= source_snapshot_.size()) {
            throw std::out_of_range("Invalid source snapshot id");
        }

        using NodeT = SourceNode<OutputPairs...>;
        using FuncT = typename NodeT::Func;

        static_assert(
            std::is_convertible_v<Fn, FuncT>,
            "Fn must be convertible to SourceNode::Func"
        );

        auto& base = source_snapshot_[snap_id];
        auto local = base->clone();
        auto source = std::static_pointer_cast<NodeT>(local);
        source->fn = FuncT(std::forward<Fn>(fn));

        schedule(local);
    }

    template<typename... OutputPairs, typename Fn>
    void add_rate_source(std::string n, double rate, Fn&& fn) {
        if (rate <= 0.0) {
            throw std::invalid_argument("rate must be > 0");
        }

        using NodeT = SourceNode<OutputPairs...>;
        using FuncT = typename NodeT::Func;

        static_assert(std::is_convertible_v<Fn, FuncT>, "Fn must match SourceNode::Func");

        auto node = NodeT::create(FuncT(std::forward<Fn>(fn)));
        node->name = std::move(n);

        connect(node);

        rate_workers_.emplace_back(RateWorker { node, rate });
    }

    void run() {
        if (running_.exchange(true, std::memory_order_acq_rel))
            return;

        for (auto& w: rate_workers_) {
            rate_threads_.emplace_back([this, w](std::stop_token st) {
                const auto period = std::chrono::duration_cast<clock::duration>(
                    std::chrono::duration<double>(1.0 / w.rate)
                );

                auto next_time = clock::now();

                while (!st.stop_requested() && running_.load(std::memory_order_acquire)) {
                    next_time += period;

                    const auto& next = w.node->execute();
                    for (auto& n: next) {
                        schedule(n);
                    }

                    auto now = clock::now();
                    if (now > next_time) {
                        next_time = now;
                        continue;
                    }

                    std::this_thread::sleep_until(next_time);
                }
            });
        }
    }

    void stop() {
        if (!running_.exchange(false, std::memory_order_acq_rel))
            return;

        for (auto& t: rate_threads_) {
            if (t.joinable())
                t.request_stop();
        }
        rate_threads_.clear();

        group_.cancel();
        group_.wait();

        AWAKENING_INFO("awakening Scheduler stopped");
    }

    void build() {
        if (built_)
            return;

        for (auto& [_, nodes]: static_tasks_snapshot_) {
            for (auto& node: nodes) {
                connect(node);
            }
        }

        for (auto& node: source_snapshot_) {
            connect(node);
        }

        for (auto& [_, nodes]: static_tasks_snapshot_) {
            for (auto& node: nodes) {
                if (node->connected_count == 0) {
                    throw std::runtime_error("Node '" + node->name + "' is isolated");
                }
            }
        }

        built_ = true;
    }

    bool is_running() const {
        return running_.load(std::memory_order_acquire);
    }

private:
    struct RateWorker {
        NodeBase::Ptr node;
        double rate;
    };

    void schedule(NodeBase::Ptr node) {
        if (!node || !running_.load(std::memory_order_acquire))
            return;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            task_queue_.push(node);
        }

        group_.run([this] { process_queue(); });
    }

    void process_queue() {
        NodeBase::Ptr node;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (task_queue_.empty())
                return;
            node = task_queue_.front();
            task_queue_.pop();
        }

        const auto& next = node->execute();
        for (auto& n: next) {
            schedule(n);
        }
    }

    void connect(NodeBase::Ptr node) {
        for (auto& tag: node->output_tags()) {
            auto it = static_tasks_snapshot_.find(tag);
            if (it == static_tasks_snapshot_.end())
                continue;

            for (auto& d: it->second) {
                node->connected_count++;
                d->connected_count++;
                node->add_downstream(tag, d->clone());
            }
        }
    }

private:
    size_t worker_count;

    std::unordered_map<std::type_index, std::vector<NodeBase::Ptr>> static_tasks_snapshot_;
    std::vector<NodeBase::Ptr> source_snapshot_;

    std::vector<RateWorker> rate_workers_;
    std::vector<std::jthread> rate_threads_;

    tbb::task_group group_;

    std::atomic<bool> running_ { false };
    bool built_ { false };

    std::queue<NodeBase::Ptr> task_queue_;
    std::mutex queue_mutex_;
};

} // namespace awakening