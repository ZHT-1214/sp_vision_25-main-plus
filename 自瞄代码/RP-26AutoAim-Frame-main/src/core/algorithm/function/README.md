# function 时间与相位工具

`src/core/algorithm/function` 提供项目中常用的时间戳转换、时间差计算、本地时间格式化和相位差归一化函数。

## API

| 函数 | 说明 | 返回单位或格式 |
| --- | --- | --- |
| `nanosecondsSinceEpoch()` | 获取当前 Unix epoch 时间 | 纳秒，`uint64_t` |
| `getNowTimestamp()` | 获取当前 Foxglove 时间戳 | `foxglove::Time` |
| `timestampMinus(t1, t2)` | 计算 `t1 - t2` | 毫秒，`double` |
| `getLocalTime()` | 获取当前本地时间 | `YYYY_M_D_H_M_S` 字符串 |
| `to_nanoseconds_since_epoch(timestamp)` | 转换 Foxglove 时间戳 | 纳秒，`uint64_t` |
| `calculate_delta_phase(new, old)` | 计算并归一化相位差 | 弧度，`[-pi, pi]` |

## 使用示例

```cpp
#include "function.hpp"

const foxglove::Time start = function::getNowTimestamp();
// ... 执行需要测量的逻辑 ...
const foxglove::Time end = function::getNowTimestamp();

const double elapsed_ms = function::timestampMinus(end, start);
const std::uint64_t timestamp_ns = function::to_nanoseconds_since_epoch(end);
const double phase_delta = function::calculate_delta_phase(new_phase, old_phase);
const std::string local_time = function::getLocalTime();
```

## 使用约定

- `nanosecondsSinceEpoch()` 和 `getNowTimestamp()` 使用系统墙上时钟，系统时间发生校准时可能跳变；需要严格单调的耗时测量时，应使用 `time` 模块提供的计时工具。
- `timestampMinus()` 按“前一个参数减后一个参数”计算，结果可以为负数。
- `calculate_delta_phase()` 使用弧度输入，返回新相位相对于旧相位的最小有向差值。
- `getLocalTime()` 使用当前系统时区，返回字符串中的字段不补前导零。

## CMake 集成

模块生成目标为 `function_lib`。其他 CMake 目标可以链接：

```cmake
target_link_libraries(your_target PRIVATE function_lib)
```

`function_lib` 会传递 Foxglove 时间类型所需的头文件依赖。
