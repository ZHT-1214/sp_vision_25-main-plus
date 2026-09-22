#include <chrono>
#include <cmath>
#include <ctime>
#include <numbers>

#include "function.hpp"

namespace
{
    constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
    constexpr double kMillisecondsPerSecond = 1'000.0;
    constexpr double kNanosecondsPerMillisecond = 1'000'000.0;
}

uint64_t function::nanosecondsSinceEpoch()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

foxglove::Time function::getNowTimestamp()
{
    const std::uint64_t now = function::nanosecondsSinceEpoch();
    return foxglove::Time(
        static_cast<std::uint32_t>(now / kNanosecondsPerSecond),
        static_cast<std::uint32_t>(now % kNanosecondsPerSecond));
}

double function::timestampMinus(const foxglove::Time &timestamp1,const foxglove::Time &timestamp2)
{
    const double seconds = static_cast<double>(timestamp1.sec()) -
                           static_cast<double>(timestamp2.sec());
    const double nanoseconds = static_cast<double>(timestamp1.nsec()) -
                               static_cast<double>(timestamp2.nsec());
    return seconds * kMillisecondsPerSecond +
           nanoseconds / kNanosecondsPerMillisecond;
}

std::string function::getLocalTime()
{
    const std::time_t now_time = std::time(nullptr);
    std::tm local_time{};
    if (localtime_r(&now_time, &local_time) == nullptr)
        return {};

    return std::to_string(local_time.tm_year + 1900) + "_" +
           std::to_string(local_time.tm_mon + 1) + "_" +
           std::to_string(local_time.tm_mday) + "_" +
           std::to_string(local_time.tm_hour) + "_" +
           std::to_string(local_time.tm_min) + "_" +
           std::to_string(local_time.tm_sec);
}

uint64_t function::to_nanoseconds_since_epoch(const foxglove::Time &timestamp)
{
    return static_cast<std::uint64_t>(timestamp.sec()) * kNanosecondsPerSecond +
           static_cast<std::uint64_t>(timestamp.nsec());
}

double function::calculate_delta_phase(const double &new_phase, const double &old_phase)
{
    constexpr double two_pi = 2.0 * std::numbers::pi;
    constexpr double pi = std::numbers::pi;
    double delta = std::fmod(new_phase - old_phase, two_pi);
    if (delta > pi)
        delta -= two_pi;
    else if (delta < -pi)
        delta += two_pi;
    return delta;
}
