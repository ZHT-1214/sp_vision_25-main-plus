#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
    ThreadPool() = default;
    ~ThreadPool();

    template<typename T>
    void addTask(T &&t);

    void run();
    void stop();

private:
    void launch_worker_once();
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::condition_variable condition_;
    std::mutex queue_mutex_;
    std::atomic_bool stop_ = false;
};

inline void ThreadPool::launch_worker_once() {
    this->workers_.emplace_back(
            [this] {
                for (;;) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex_);
                        this->condition_.wait(lock, [this] { return this->stop_ || !this->tasks_.empty(); });
                        if (this->stop_ && this->tasks_.empty()) {
                            return;// 如果停止标志为 true 且任务队列为空，退出线程
                        }
                        task = std::move(this->tasks_.front());
                        this->tasks_.pop();
                    }
                    task();
                }
            });
}

inline void ThreadPool::run() {
    for (size_t i = 0; i < std::thread::hardware_concurrency(); ++i) {
        launch_worker_once();
    }
}

inline void ThreadPool::stop() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    condition_.notify_all();
    for (std::thread &worker: workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

inline ThreadPool::~ThreadPool() {
    stop();
}

template<typename T>
void ThreadPool::addTask(T &&t) {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    if (this->stop_) {
        throw std::runtime_error("thread pool already stopped");
    }

    this->tasks_.emplace(std::forward<T>(t));
    this->condition_.notify_one();
}
