#pragma once

#include <cstdint>
#include <string>

#include "Time_generated.h"

namespace function
{
    /**
     * @brief 获取当前 Unix epoch 时间，单位为纳秒。
     *
     * 使用系统墙上时钟（`std::chrono::system_clock`），适合生成跨模块传递的
     * 时间戳，不适合直接用于需要单调递增保证的耗时测量。
     *
     * @return 自 Unix epoch 起经过的纳秒数。
     */
    uint64_t nanosecondsSinceEpoch();

    /**
     * @brief 获取当前时间并转换为 Foxglove 时间戳。
     *
     * @return 当前时间，秒和纳秒分别存储在 `foxglove::Time` 中。
     */
    foxglove::Time getNowTimestamp();

    /**
     * @brief 计算两个 Foxglove 时间戳的差值。
     *
     * @param timestamp1 被减时间戳。
     * @param timestamp2 减数时间戳。
     * @return `timestamp1 - timestamp2`，单位为毫秒；结果可以为负数。
     */
    double timestampMinus(const foxglove::Time &timestamp1,const foxglove::Time &timestamp2);

    /**
     * @brief 获取当前本地时间的字符串表示。
     *
     * @return 格式为 `YYYY_M_D_H_M_S` 的本地时间字符串，日期和时间字段不补零。
     */
    std::string getLocalTime();

    /**
     * @brief 将 Foxglove 时间戳转换为 Unix epoch 纳秒数。
     *
     * @param timestamp 待转换的 Foxglove 时间戳。
     * @return 自 Unix epoch 起经过的纳秒数。
     */
    uint64_t to_nanoseconds_since_epoch(const foxglove::Time &timestamp);

    /**
     * @brief 计算两个相位之间的最小有向差值。
     *
     * 结果会归一化到 `[-pi, pi]`，并按照“新相位减旧相位”计算。
     *
     * @param new_phase 新相位，单位为弧度。
     * @param old_phase 旧相位，单位为弧度。
     * @return 归一化后的相位差，单位为弧度。
     */
    double calculate_delta_phase(const double &new_phase, const double &old_phase);
}
