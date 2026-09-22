#pragma once

#include "latest_channel/latest_channel.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <ostream>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

struct LatestChannelBenchmarkConfig
{
    // 订阅者线程数量。默认使用硬件并发数 - 1，给发布线程留出一个逻辑核心。
    std::size_t subscriber_count = [] {
        const unsigned int concurrency = std::thread::hardware_concurrency();
        return concurrency > 1 ? static_cast<std::size_t>(concurrency - 1) : std::size_t{1};
    }();
    // 发布次数。不包含 benchmark 结束时用于唤醒订阅者退出的额外一次 publish。
    // 当 publish_duration > 0 时，这个字段不再限制发布次数，只用于固定次数模式。
    std::uint64_t publish_count = 100000;
    // 固定时长发布模式。为 0 时使用固定 publish_count；非 0 时持续发布到指定时长结束。
    std::chrono::milliseconds publish_duration{0};
    // 订阅者线程全部就绪后，发布线程开始前额外等待的时间。
    std::chrono::milliseconds subscriber_settle_time{10};
    // 发布完成后留给订阅者继续消费最新消息的时间。
    std::chrono::milliseconds drain_time{20};
    // 每个订阅者收到一条消息后执行的模拟工作量。0 表示不额外模拟耗时任务。
    std::uint64_t subscriber_work_iterations = 0;
    // true: 预先构造所有 shared_ptr<T>，尽量隔离 LatestChannel 自身开销。
    // false: 把每次消息构造和 make_shared<T> 的成本也计入发布耗时。
    bool prebuild_messages = false;
    // 固定时长模式下预构造消息的循环池大小。固定次数模式会预构造 publish_count + 1 条。
    std::size_t prebuild_pool_size = 65536;
    // 是否在 benchmark 完成后向 stdout 打印汇总结果。
    bool print_summary = true;
};

struct LatestChannelBenchmarkResult
{
    // typeid(T).name() 的原始类型名，具体可读性取决于编译器。
    std::string type_name;
    // sizeof(T)，只反映对象本体大小，不包含堆上额外资源。
    std::size_t value_size_bytes = 0;
    // 本次测试实际启动的订阅者线程数量。
    std::size_t subscriber_count = 0;
    // 本次测试统计的发布次数。
    std::uint64_t publish_count = 0;
    // true 表示本次使用固定时长发布模式；false 表示固定次数发布模式。
    bool fixed_duration_mode = false;
    // 固定时长模式下的目标发布时长。固定次数模式下为 0。
    double target_publish_duration_ms = 0.0;
    // 所有订阅者合计收到的新消息次数。latest-only 语义下允许小于 publish_count * subscriber_count。
    std::uint64_t total_received = 0;
    // 单个订阅者收到消息次数的最小值。
    std::uint64_t min_received_per_subscriber = 0;
    // 单个订阅者收到消息次数的最大值。
    std::uint64_t max_received_per_subscriber = 0;
    // 每个订阅者平均收到的新消息次数。
    double average_received_per_subscriber = 0.0;
    // 所有订阅者合计执行的模拟工作校验值，用于避免编译器完全优化掉订阅者工作负载。
    std::uint64_t subscriber_work_checksum = 0;
    // total_received / (publish_count * subscriber_count)，用于观察订阅者跳帧比例。
    double receive_ratio = 0.0;
    // 只统计 publish 循环耗时，不包含线程创建、启动等待、drain 和 join。
    double publish_duration_ms = 0.0;
    // 整个 benchmark 耗时，包含线程创建、启动等待、publish、drain 和 join。
    double total_duration_ms = 0.0;
    // publish_count / publish_duration，表示发布侧吞吐。
    double publish_per_second = 0.0;
    // publish 循环中每次消息构造 + LatestChannel::publish 的平均耗时。
    double average_publish_ns = 0.0;
    // prebuild_messages == false 时，每次消息构造/分配的平均耗时；预构造模式下为 0。
    double average_message_build_ns = 0.0;
    // prebuild_messages == false 时，扣除消息构造后的 LatestChannel::publish 平均耗时；预构造模式下等于 average_publish_ns。
    double average_channel_publish_ns = 0.0;
    // average_message_build_ns / average_publish_ns，用于观察构造/分配占总发布耗时的比例。
    double message_build_ratio = 0.0;
    // 记录本次测试是否启用了消息预构造。
    bool prebuild_messages = false;
    // 本次测试实际预构造的消息数量。未启用预构造时为 0。
    std::size_t prebuild_pool_size = 0;
    // 记录本次测试每条订阅消息执行的模拟工作量。
    std::uint64_t subscriber_work_iterations = 0;
};

inline void print_latest_channel_benchmark_result(const LatestChannelBenchmarkResult &result,
                                                  std::ostream &os = std::cout)
{
    const auto old_flags = os.flags();
    const auto old_precision = os.precision();
    const auto old_fill = os.fill();

    auto name_column = [&](const char *name) {
        os << "  " << std::left << std::setw(24) << name << std::right << ' ';
    };
    auto separator = [&] {
        os << "  " << std::string(56, '-') << '\n';
    };

    os << "\n[LatestChannelBenchmark]\n";
    name_column("数据类型");
    os << result.type_name << '\n';
    name_column("对象大小");
    os << std::setw(14) << result.value_size_bytes << " B\n";
    name_column("订阅者数量");
    os << std::setw(14) << result.subscriber_count << '\n';
    name_column("发布模式");
    os << std::setw(14) << (result.fixed_duration_mode ? "固定时长" : "固定次数") << '\n';
    if (result.fixed_duration_mode)
    {
        name_column("目标发布时长");
        os << std::fixed << std::setprecision(3) << std::setw(14)
           << result.target_publish_duration_ms << " ms\n";
    }
    name_column("实际发布次数");
    os << std::setw(14) << result.publish_count << '\n';
    name_column("订阅者工作量");
    os << std::setw(14) << result.subscriber_work_iterations << " iter/msg\n";
    name_column("预构造消息");
    os << std::setw(14) << std::boolalpha << result.prebuild_messages << '\n';
    if (result.prebuild_messages)
    {
        name_column("预构造池大小");
        os << std::setw(14) << result.prebuild_pool_size << '\n';
    }

    separator();
    os << std::fixed << std::setprecision(3);
    name_column("发布耗时");
    os << std::setw(14) << result.publish_duration_ms << " ms\n";
    name_column("总耗时");
    os << std::setw(14) << result.total_duration_ms << " ms\n";
    os << std::setprecision(0);
    name_column("发布吞吐");
    os << std::setw(14) << result.publish_per_second << " msg/s\n";
    os << std::setprecision(1);
    name_column("平均总发布耗时");
    os << std::setw(14) << result.average_publish_ns << " ns"
       << " (" << std::fixed << std::setprecision(6)
       << result.average_publish_ns / 1000000.0 << " ms)\n";
    os << std::setprecision(1);
    name_column("平均构造耗时");
    os << std::setw(14) << result.average_message_build_ns << " ns"
       << " (" << std::fixed << std::setprecision(6)
       << result.average_message_build_ns / 1000000.0 << " ms, "
       << std::setprecision(2) << result.message_build_ratio * 100.0 << "%)\n";
    os << std::setprecision(1);
    name_column("平均Channel发布耗时");
    os << std::setw(14) << result.average_channel_publish_ns << " ns"
       << " (" << std::fixed << std::setprecision(6)
       << result.average_channel_publish_ns / 1000000.0 << " ms)\n";

    separator();
    name_column("总接收次数");
    os << std::setw(14) << result.total_received << '\n';
    os << std::fixed << std::setprecision(3);
    name_column("接收比例");
    os << std::setw(13) << result.receive_ratio * 100.0 << "%\n";
    os << std::setprecision(1);
    name_column("单订阅者平均接收");
    os << std::setw(14) << result.average_received_per_subscriber << '\n';
    name_column("单订阅者最少接收");
    os << std::setw(14) << result.min_received_per_subscriber << '\n';
    name_column("单订阅者最多接收");
    os << std::setw(14) << result.max_received_per_subscriber << '\n';
    name_column("工作校验值");
    os << std::setw(14) << result.subscriber_work_checksum << '\n';

    os.flags(old_flags);
    os.precision(old_precision);
    os.fill(old_fill);
}

namespace latest_channel_benchmark_detail
{
    template <typename T, typename Factory>
    typename LatestChannel<T>::DataPtr make_data_ptr(Factory &factory, std::uint64_t index)
    {
        using Result = std::invoke_result_t<Factory &, std::uint64_t>;

        if constexpr (std::is_convertible_v<Result, typename LatestChannel<T>::DataPtr>)
        {
            return std::invoke(factory, index);
        }
        else
        {
            return std::make_shared<T>(std::invoke(factory, index));
        }
    }

    template <typename ClockDuration>
    double to_ms(ClockDuration duration)
    {
        return std::chrono::duration<double, std::milli>(duration).count();
    }

    inline std::uint64_t run_subscriber_work(std::uint64_t iterations,
                                             std::uint64_t seed)
    {
        std::uint64_t value = seed + 0x9e3779b97f4a7c15ULL;
        for (std::uint64_t i = 0; i < iterations; ++i)
        {
            value ^= value >> 12;
            value ^= value << 25;
            value ^= value >> 27;
            value *= 0x2545f4914f6cdd1dULL;
        }
        return value;
    }
}

template <typename T>
struct LatestChannelDefaultFactory
{
    T operator()(std::uint64_t) const
    {
        return T{};
    }
};

template <typename T, typename Factory>
LatestChannelBenchmarkResult benchmark_latest_channel(const LatestChannelBenchmarkConfig &config,
                                                      Factory &&factory_arg)
{
    using Clock = std::chrono::steady_clock;
    using DataPtr = typename LatestChannel<T>::DataPtr;

    LatestChannel<T> channel;
    auto factory = std::forward<Factory>(factory_arg);
    const bool fixed_duration_mode = config.publish_duration > std::chrono::milliseconds{0};

    std::vector<DataPtr> prebuilt_messages;
    if (config.prebuild_messages)
    {
        const std::size_t prebuild_count =
            fixed_duration_mode
                ? std::max<std::size_t>(config.prebuild_pool_size, 1)
                : static_cast<std::size_t>(config.publish_count + 1);

        prebuilt_messages.reserve(prebuild_count);
        for (std::size_t i = 0; i < prebuild_count; ++i)
        {
            prebuilt_messages.emplace_back(
                latest_channel_benchmark_detail::make_data_ptr<T>(factory, static_cast<std::uint64_t>(i)));
        }
    }

    auto make_message = [&](std::uint64_t index) -> DataPtr {
        if (config.prebuild_messages)
        {
            return prebuilt_messages[static_cast<std::size_t>(index % prebuilt_messages.size())];
        }

        return latest_channel_benchmark_detail::make_data_ptr<T>(factory, index);
    };

    std::atomic_bool start{false};
    std::atomic_bool stop{false};
    std::atomic_size_t ready_count{0};
    std::vector<std::uint64_t> received_counts(config.subscriber_count, 0);
    std::vector<std::uint64_t> subscriber_work_checksums(config.subscriber_count, 0);
    std::vector<std::thread> subscribers;
    subscribers.reserve(config.subscriber_count);

    const auto total_begin = Clock::now();

    for (std::size_t subscriber_index = 0; subscriber_index < config.subscriber_count; ++subscriber_index)
    {
        subscribers.emplace_back([&, subscriber_index] {
            Subscriber<T> subscriber(channel);
            ready_count.fetch_add(1, std::memory_order_release);

            while (!start.load(std::memory_order_acquire))
            {
                std::this_thread::yield();
            }

            std::uint64_t local_received = 0;
            std::uint64_t local_work_checksum = 0;
            while (!stop.load(std::memory_order_acquire))
            {
                DataPtr data = subscriber.wait_next();
                if (stop.load(std::memory_order_acquire))
                {
                    break;
                }
                if (data)
                {
                    ++local_received;
                    if (config.subscriber_work_iterations > 0)
                    {
                        local_work_checksum ^=
                            latest_channel_benchmark_detail::run_subscriber_work(
                                config.subscriber_work_iterations,
                                subscriber.received_seq() + subscriber_index);
                    }
                }
            }

            received_counts[subscriber_index] = local_received;
            subscriber_work_checksums[subscriber_index] = local_work_checksum;
        });
    }

    while (ready_count.load(std::memory_order_acquire) < config.subscriber_count)
    {
        std::this_thread::yield();
    }

    start.store(true, std::memory_order_release);
    std::this_thread::sleep_for(config.subscriber_settle_time);

    const auto publish_begin = Clock::now();
    std::uint64_t actual_publish_count = 0;
    typename Clock::duration total_message_build_duration{};
    if (fixed_duration_mode)
    {
        const auto publish_deadline = publish_begin + config.publish_duration;
        while (Clock::now() < publish_deadline)
        {
            const auto build_begin = Clock::now();
            DataPtr data = make_message(actual_publish_count);
            const auto build_end = Clock::now();
            total_message_build_duration += build_end - build_begin;
            channel.publish(data);
            ++actual_publish_count;
        }
    }
    else
    {
        for (; actual_publish_count < config.publish_count; ++actual_publish_count)
        {
            const auto build_begin = Clock::now();
            DataPtr data = make_message(actual_publish_count);
            const auto build_end = Clock::now();
            total_message_build_duration += build_end - build_begin;
            channel.publish(data);
        }
    }
    const auto publish_end = Clock::now();

    std::this_thread::sleep_for(config.drain_time);
    stop.store(true, std::memory_order_release);
    channel.publish(make_message(actual_publish_count));

    for (std::thread &subscriber : subscribers)
    {
        if (subscriber.joinable())
        {
            subscriber.join();
        }
    }

    const auto total_end = Clock::now();

    LatestChannelBenchmarkResult result;
    result.type_name = typeid(T).name();
    result.value_size_bytes = sizeof(T);
    result.subscriber_count = config.subscriber_count;
    result.publish_count = actual_publish_count;
    result.fixed_duration_mode = fixed_duration_mode;
    result.target_publish_duration_ms = latest_channel_benchmark_detail::to_ms(config.publish_duration);
    result.prebuild_messages = config.prebuild_messages;
    result.prebuild_pool_size = prebuilt_messages.size();
    result.subscriber_work_iterations = config.subscriber_work_iterations;

    result.total_received = std::accumulate(received_counts.begin(), received_counts.end(), std::uint64_t{0});
    result.subscriber_work_checksum =
        std::accumulate(subscriber_work_checksums.begin(), subscriber_work_checksums.end(), std::uint64_t{0},
                        [](std::uint64_t lhs, std::uint64_t rhs) {
                            return lhs ^ rhs;
                        });
    if (!received_counts.empty())
    {
        const auto minmax = std::minmax_element(received_counts.begin(), received_counts.end());
        result.min_received_per_subscriber = *minmax.first;
        result.max_received_per_subscriber = *minmax.second;
        result.average_received_per_subscriber =
            static_cast<double>(result.total_received) / static_cast<double>(received_counts.size());
    }

    const double max_possible_received =
        static_cast<double>(result.publish_count) * static_cast<double>(config.subscriber_count);
    if (max_possible_received > 0.0)
    {
        result.receive_ratio = static_cast<double>(result.total_received) / max_possible_received;
    }

    result.publish_duration_ms = latest_channel_benchmark_detail::to_ms(publish_end - publish_begin);
    result.total_duration_ms = latest_channel_benchmark_detail::to_ms(total_end - total_begin);
    if (result.publish_duration_ms > 0.0)
    {
        result.publish_per_second =
            static_cast<double>(result.publish_count) / (result.publish_duration_ms / 1000.0);
    }
    if (result.publish_count > 0)
    {
        result.average_publish_ns =
            (result.publish_duration_ms * 1000000.0) / static_cast<double>(result.publish_count);
        result.average_message_build_ns =
            (latest_channel_benchmark_detail::to_ms(total_message_build_duration) * 1000000.0) /
            static_cast<double>(result.publish_count);
        result.average_channel_publish_ns = result.average_publish_ns - result.average_message_build_ns;
        if (result.average_publish_ns > 0.0)
        {
            result.message_build_ratio = result.average_message_build_ns / result.average_publish_ns;
        }
    }

    if (config.print_summary)
    {
        print_latest_channel_benchmark_result(result);
    }

    return result;
}

template <typename T>
LatestChannelBenchmarkResult benchmark_latest_channel(
    const LatestChannelBenchmarkConfig &config = LatestChannelBenchmarkConfig{})
{
    return benchmark_latest_channel<T>(config, LatestChannelDefaultFactory<T>{});
}
