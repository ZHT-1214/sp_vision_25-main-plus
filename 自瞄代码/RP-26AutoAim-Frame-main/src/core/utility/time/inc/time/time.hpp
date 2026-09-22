#pragma once
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <iostream>
#include <array>
#include <vector>

namespace timetool::detail
{

template <typename T>
concept DurationUnit = 
    std::same_as<T, std::chrono::nanoseconds> ||
    std::same_as<T, std::chrono::microseconds> ||
    std::same_as<T, std::chrono::milliseconds> ||
    std::same_as<T, std::chrono::seconds> ||
    std::same_as<T, std::chrono::minutes> ||
    std::same_as<T, std::chrono::hours>;

template <DurationUnit Target, DurationUnit...Units>
inline consteval std::size_t duration_unit_index()
{
    // static_assert 是 false 触发编译报错
    static_assert(sizeof...(Units) != 0, "The DurationUnit pack is null!");
    
    std::array<bool, sizeof...(Units)> matches = { std::same_as<Target, Units>... };
    for (std::size_t i = 0; i < matches.size(); ++i)
            if (matches[i])
                return i; // 找到后直接返回

    return sizeof...(Units); // 没找到时返回包的大小
}

template <DurationUnit Unit>
inline consteval const char* duration_suffix()
{
    constexpr const char* suffixes[] = {
        "ns", "µs", "ms", "s", "min", "h"
    };
    return suffixes[detail::duration_unit_index<
        Unit,
        std::chrono::nanoseconds,
        std::chrono::microseconds,
        std::chrono::milliseconds,
        std::chrono::seconds,
        std::chrono::minutes,
        std::chrono::hours>()];
}

} // namespace timetool::detail

// 工具函数 
namespace timetool
{
    // 从系统时钟（std::chrono::system_clock）获取的时间戳，精度为纳秒
    using Timestamp = std::chrono::time_point<std::chrono::system_clock>;

    /**
     * @brief 获取当前时间戳（纳秒）
     * @return 当前时间的纳秒时间戳
     */
    inline Timestamp now()
    {
        return std::chrono::system_clock::now();
    }

    /**
     * @brief 将统一时间戳转换为 epoch nanoseconds。
     * @param timestamp 统一时间戳
     * @return 从 epoch 开始的纳秒数
     */
    inline uint64_t to_epoch_nanoseconds(Timestamp timestamp)
    {
        return std::chrono::nanoseconds(timestamp.time_since_epoch()).count();
    }

    /**
     * @brief 计算两个时间戳的差值（前减后）
     * @tparam Duration std::chrono::duration 类型（如 milliseconds、seconds）
     * @param timestamp1 被减数时间戳（纳秒）
     * @param timestamp2 减数时间戳（纳秒）
     * @return 差值，转换为 Duration 单位的数值
     */
    template <timetool::detail::DurationUnit Unit>
    inline double minus(Timestamp timestamp1, Timestamp timestamp2)
    {
        auto diff = timestamp1 - timestamp2;
        return std::chrono::duration<double, typename Unit::period>(diff).count();
    }

    /**
     * @brief 计算两个时间戳的差值（前减后），单位为毫秒
     * @param timestamp1 被减数时间戳（纳秒）
     * @param timestamp2 减数时间戳（纳秒）
     * @return 时间差（毫秒）
     */
    inline double minus_ms(Timestamp timestamp1, Timestamp timestamp2)
    {
        return minus<std::chrono::milliseconds>(timestamp1, timestamp2);
    }
}

// 工具类
namespace timetool
{

class Timer
{
public:
    enum State
    {
        NotReady,
        Timing,
        Stopped
    };

public:
    inline void start()
    {
        if (m_state == Timing)
            throw std::logic_error("The timer is currently working!");

        m_start = now();
        m_state = Timing;
    }

    inline void stop()
    {
        if (m_state != Timing)
            throw std::logic_error("Failed to stop the timer without starting it!");

        m_stop = now();
        m_state = Stopped;
    }

    template<detail::DurationUnit Unit>
    inline void print()
    {
        if (m_state != Stopped)
            throw std::logic_error("Failed to print before stopping the timer");

        std::cout
            << "Timing result: "
            << minus<Unit>(m_stop, m_start)
            << " " << detail::duration_suffix<Unit>() << std::endl;
    }
    
private:
    State m_state = NotReady; // Timer 可能用于高性能计时，为避免 cache 命中率下降，两个 Timestamp 不使用 std::optional，故引入该状态变量
    Timestamp m_start;
    Timestamp m_stop;
};

template <std::size_t sample_count>
class PerfAnalyzer
{
    static_assert(sample_count > 1, "PerfAnalyzer: sample_count must be > 1 to calculate intervals.");
public:
    inline PerfAnalyzer() : m_end_index(0)
    {
        m_samples.reserve(sample_count);
    };
    inline void sample(const Timestamp &stamp)
    {
        if (m_samples.size() != sample_count)
        {
            m_samples.push_back(stamp);
            m_end_index += 1;
        }
        else
        {
            m_end_index %= sample_count;
            m_samples[m_end_index] = stamp;
            m_end_index += 1;
        }
    }
    template<detail::DurationUnit Unit>
    void try_print_precise()
    {
        if (m_samples.size() != sample_count)
            return;

        constexpr double mean_x = (sample_count - 1) / 2.0;
        // 编译期计算最小二乘分母
        constexpr double ols_denominator = sample_count * (sample_count * sample_count - 1.0) / 12.0;
        // 编译期生成权重表 (x_i - mean_x)
        // 使用立即调用函数表达式 (IIFE) 在编译期初始化 std::array
        constexpr std::array<double, sample_count> x_weights = []() {
            std::array<double, sample_count> weights{};
            for (std::size_t i = 0; i < sample_count; ++i)
                weights[i] = static_cast<double>(i) - mean_x;
            return weights;
        }();

        double numerator = 0.0;

        // 选定最早的点 t0 作为零点，防止 double 精度截断
        const Timestamp t0 = m_samples[m_end_index % sample_count];

        for (std::size_t i = 0; i < sample_count; ++i)
        {
            const std::size_t current_idx = (m_end_index + i) % sample_count;
            double y_i = minus<Unit>(m_samples[current_idx], t0);

            // weight_i 直接查编译期生成的表，省去每次的减法
            numerator += y_i * x_weights[i];
        }

        double avg = numerator / ols_denominator;
        std::cout
            << "precise average time interval: "
            << avg << " " << detail::duration_suffix<Unit>() << '\n';
    }
    template<detail::DurationUnit Unit>
    void try_print_rough()
    {
        if (m_samples.size() != sample_count)
            return;

        const std::size_t oldest_idx = m_end_index % sample_count;
        const std::size_t newest_idx = (m_end_index + sample_count - 1) % sample_count;

        double avg = minus<Unit>(m_samples[newest_idx], m_samples[oldest_idx]) / (sample_count - 1);

        std::cout
            << "rough average time interval: "
            << avg << " " << detail::duration_suffix<Unit>() << '\n';
    }
private:
    std::size_t m_end_index;
    std::vector<Timestamp> m_samples;
};

} // namespace timetool
