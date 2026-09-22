# 开环轨迹规划

本文描述 `src/tasks/base/dta_utils.hpp` 中 `LimitTrajectory` 的设计与实现。该模块用于自动瞄准和能量机关控制，将目标预测得到的离散 `yaw/pitch` 控制点转换成满足云台加速度约束的连续开环前馈轨迹。

相关代码主要位于：

- `src/tasks/base/dta_utils.hpp`
- `src/tasks/base/traj.hpp`
- `src/tasks/auto_aim/armor_control/very_aimer.cpp`
- `src/tasks/auto_buff/rune_control/rune_aimer.cpp`

## 问题背景

自动瞄准和能量机关控制中都会出现目标控制点突变：

- 自动瞄准切换装甲板时，命中点对应的 `yaw/pitch` 会发生不连续跳变。
- 能量机关预测命中点随相位推进，未来一段时间内需要给出可执行的云台控制轨迹。
- 如果直接把瞬时目标点发给下位机，云台控制器会在切换点承受过大的速度和加速度需求，表现为跟随抖动、超调或开火窗口变窄。

`LimitTrajectory` 的目标不是求解全局最优控制问题，而是在保持在线计算轻量的前提下，把离散射击轨迹转换为一条位置、速度、加速度连续，并且尽量满足最大加速度约束的开环前馈轨迹。

## 输入与输出

上层控制器会先采样一段目标射击轨迹。每个采样点由目标状态预测、弹道解算和目标选择共同确定，得到一个 `GimbalState`：

```cpp
struct GimbalState {
    State yaw_state;
    State pitch_state;
    int aim_id;
};
```
 
其中每个轴的状态为：

```cpp
struct State {
    double p;       // 位置：yaw 或 pitch
    double v;       // 速度
    double a;       // 加速度
    bool on_traj;   // 是否位于正常目标轨迹段
};
```

采样轨迹保存在 `Trajectory<GimbalState, double>` 中，时间轴使用 `double`。`LimitTrajectory` 继承自该轨迹容器，并额外维护两条五次多项式轨迹：

```cpp  
Traj yaw_traj;
Traj pitch_traj;
```

查询时调用：

```cpp
auto control = limit_traj.state_at(t);
```

返回的 `control` 包含当前时刻的 `yaw/pitch` 位置、速度和加速度，可作为上位机给下位机的前馈控制指令。

## 总体思路

`LimitTrajectory` 借鉴类 MINCO 的参数化思想，但实现上更轻量：

- 控制点位置不作为优化变量，由目标预测和弹道解算直接给定。
- 相邻控制点之间使用五次多项式连接。
- 每段多项式系数由边界位置、速度、加速度闭式求解。
- 在线只调整切换目标附近的时间覆盖范围，用更长时间分摊位置跳变，从而降低加速度。

也就是说，它不是在线求解 QP/MPC，而是“固定几何点 + 解析多项式 + 局部时间扩展”。

## 五次多项式段

每个轴独立规划。对于一段持续时间为 `T` 的轨迹，位置写成：

```text
p(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5
```

五次多项式可以同时满足起点和终点的位置、速度、加速度约束：

```text
p(0), v(0), a(0), p(T), v(T), a(T)
```

代码中由 `QuinticSegment::solve1d_closed_form()` 直接计算 `c0` 到 `c5`。其中：

```text
c0 = p0
c1 = v0
c2 = 0.5 * a0
```

剩余高阶项由终点边界误差闭式求解：

```text
dp = p1 - (p0 + v0 T + 0.5 a0 T^2)
dv = v1 - (v0 + a0 T)
da = a1 - a0
```

```text
c3 = (10 dp - 4 dv T + 0.5 da T^2) / T^3
c4 = (-15 dp + 7 dv T - da T^2) / T^4
c5 = (6 dp - 3 dv T + 0.5 da T^2) / T^5
```

因此每个段天然具有位置、速度和加速度连续性。构造完成后，`QuinticSegment::eval(t)` 可以返回任意时刻的 `p/v/a`。

## 节点速度与加速度估计

输入控制点只包含位置和 `aim_id`，速度和加速度需要根据邻接段估计。

`estimate_knot_states()` 会先计算每个正常段的平均速度：

```text
v_avg = (p_r - p_l) / (t_r - t_l)
```

对于同时拥有左段和右段的内部节点，速度取左右平均速度的时间加权平均：

```text
v_i = (T_right * v_left + T_left * v_right) / (T_left + T_right)
```

加速度使用左右平均速度差估计：

```text
a_i = 2 * (v_right - v_left) / (T_left + T_right)
```

端点或被切换区间隔开的节点没有完整邻接信息时，默认速度和加速度为 0。这样可以在普通连续段上获得较平滑的节点状态，同时避免在目标切换处把突变错误地传播为巨大速度。

## 角度展开

`yaw` 和 `pitch` 都是角度量，直接插值会遇到 `+-pi` 跳变。构建轨迹前，`unwrap_states()` 会沿时间顺序调用：

```cpp
angles::unwrap_angle(prev, current)
```

它把后一个角度展开到与前一个角度连续的分支上，避免轨迹从 `179 deg` 到 `-179 deg` 时走一圈大弧。

增量追加轨迹时，`unwrap_appended_states()` 只处理新增部分，避免重复展开历史点。

## 切换区间检测

自动瞄准切换装甲板时，`aim_id` 会变化。`find_nearest_change_interval()` 会扫描控制点序列，找到距离当前时间最近的 `aim_id` 变化段：

```text
if cp[i].aim_id != cp[i + 1].aim_id:
    candidate_interval = [i, i + 1]
```

如果没有发生目标切换，则所有相邻采样点都按普通五次多项式段连接。

如果发现切换段，`LimitTrajectory` 不直接用原始相邻点连接，而是把该段标记为需要限加速度处理的特殊区间。

## 切换区间扩展

切换目标时，位置跳变集中在很短时间内会导致加速度过大。`expand_limit_interval()` 会以原始切换段为中心，向左右扩展覆盖范围：

```text
base interval: [base_l, base_r]
expanded interval: [l, r]
```

扩展时有两个限制：

- 左侧最多扩展到切换前同一 `aim_id` 连续区间的中点附近。
- 右侧最多扩展到切换后同一 `aim_id` 连续区间的中点附近。

这样可以避免过渡段吞掉过多正常跟踪区间。扩展后的区间仍连接原来的两个目标控制点集合，但位置变化被分摊到更长时间内完成。

每次尝试扩展半径时，代码会构造一个从 `l` 到 `r` 的五次多项式段，并检查该段最大加速度：

```text
max |a(t)| <= max_acc
```

若原始切换段已经满足约束，就不扩展；否则逐步增大半径，直到满足最大加速度约束或达到允许扩展边界。

## 解析最大加速度

`QuinticSegment::max_abs_acc()` 不靠固定步长采样估计最大加速度，而是解析检查。

五次位置多项式的加速度为三次函数：

```text
a(t) = 2c2 + 6c3 t + 12c4 t^2 + 20c5 t^3
```

加速度极值只可能出现在区间端点，或 jerk 为 0 的内部点。jerk 为：

```text
j(t) = da(t)/dt = 6c3 + 24c4 t + 60c5 t^2
```

因此代码检查：

- `t = 0`
- `t = T`
- 二次方程 `60c5 t^2 + 24c4 t + 6c3 = 0` 在 `(0, T)` 内的实根

再取这些候选点上的最大绝对加速度。相比离散采样，这种方式更精确，也更适合高频在线规划。

## 分段构造

`make_segment_descs()` 会把控制点序列转换为若干 `SegmentDesc`：

```cpp
struct SegmentDesc {
    int l;
    int r;
    bool on_traj;
};
```

普通段为相邻点：

```text
[i, i + 1], on_traj = true
```

切换过渡段为扩展后的大区间：

```text
[limit_l, limit_r], on_traj = false
```

随后 `build_continuous_centered_traj()` 会：

1. 根据这些分段估计节点速度和加速度。
2. 对每个分段调用 `QuinticSegment::build()`。
3. 记录每个段对应的原始控制点索引。
4. 重建 `seg_prefix_time`，用于快速按时间查询。

`on_traj = false` 的段表示这段是为了限加速度生成的过渡段，不一定严格位于原始目标射击轨迹上。控制器仍可查询原始目标轨迹用于开火判断。

## yaw/pitch 双轴处理

`build_limit()` 会分别对 `yaw` 和 `pitch` 调用 `limit_traj()`：

```cpp
limit_traj(yaw_traj,   ..., max_yaw_acc,   project_yaw)
limit_traj(pitch_traj, ..., max_pitch_acc, project_pitch)
```

两轴共用同一个最近切换区间，但分别使用自己的最大加速度约束。这样可以适配云台 yaw 轴和 pitch 轴不同的机械能力。

查询时，`state_at(t)` 分别查询两条多项式轨迹，再组合成一个 `GimbalState`：

```cpp
GimbalState::State yaw   = state_at(t, yaw_traj);
GimbalState::State pitch = state_at(t, pitch_traj);
return GimbalState(yaw, pitch);
```

## 增量更新

控制循环中，如果目标状态没有发生变化，上层不会每帧重建整条轨迹，而是让时间向前推进，并在需要时追加未来采样点。

`update_limit_after_append()` 用于处理这种场景：

1. 如果轨迹太短或历史规划不存在，直接重建。
2. 对新增控制点做角度展开。
3. 检查新的切换区间或旧的限加速度区间是否触及尾部。
4. 若触及尾部，说明新增点可能影响过渡段，直接重建。
5. 否则只删除尾部少量段，并用普通五次多项式追加新尾段。

这种增量逻辑减少了重复计算，也避免已有过渡段在每个控制周期被无意义地重建。

## 与原始目标轨迹的关系

上层控制器通常同时维护两条轨迹：

- 原始目标轨迹：由目标预测、弹道解算、目标选择直接得到，表示“应该打哪里”。
- 控制轨迹：由 `LimitTrajectory` 生成，表示“云台应该怎么平滑地转过去”。

自动瞄准中，控制输出来自 `LimitTrajectory::state_at(t)`，而开火判断还会参考原始目标轨迹和目标点：

```text
target trajectory: 一定指向预测命中点
control trajectory: 满足加速度约束的平滑控制轨迹
```

如果发射延迟区间内控制轨迹与原始目标轨迹偏差过大，即使当前瞬间看起来接近目标，也可以提前禁止开火。这使开火逻辑从单点阈值判断转向轨迹一致性判断。

## 工程特点

`LimitTrajectory` 的特点可以概括为：

- 不求解通用优化问题，五次多项式系数闭式计算。
- 只在切换目标附近调整时间覆盖范围，控制点位置保持不变。
- 解析计算最大加速度，避免采样误差。
- yaw/pitch 分轴限幅，适配不同机械能力。
- 支持增量追加，减少高频控制循环中的重复构建开销。
- 输出位置、速度和加速度，方便下位机叠加前馈控制。

从控制角度看，它是一种轻量级开环前馈轨迹规划器。它不替代下位机闭环控制，而是为下位机提供一条更平滑、更符合云台能力的参考轨迹，从而降低切板和打符时的控制压力。
