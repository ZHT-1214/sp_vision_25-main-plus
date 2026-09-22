# 状态估计

本文描述 AWAKENING 中自动瞄准与能量机关的状态估计设计。相关代码主要位于：

- `src/tasks/auto_aim/armor_track/`
- `src/tasks/auto_buff/rune_track/`
- `3rdparty/KalmanHyLib/error_state_extended_kalman_filter.hpp`

本项目的估计模块不是简单地对单个检测框做 PnP 后滤波，而是尽量把 RoboMaster 目标的结构先验纳入状态空间和观测模型中。自动瞄准链路维护“整车”级状态，统一利用装甲板、灯条和深度差观测；能量机关链路维护能量机关平面状态，统一利用 R 标、扇叶和靶盘观测。两者都使用误差状态扩展卡尔曼滤波器，在预测、匹配和更新阶段保持几何约束的一致性。

## 设计目标

RoboMaster 视觉估计面对的主要问题包括：

- 单帧有效观测偏少。目标高速旋转、侧身或被遮挡时，完整装甲板可能只有一块可用，但相邻单灯条仍包含位置、尺度和朝向信息。
- PnP 位姿容易抖动。远距离小目标、角点回归误差、斜视角和共面矩形目标都会让深度和姿态估计不稳定。
- 目标运动不是纯二维 yaw。地面坡度、云台外参误差、车体姿态变化、前哨站和基地结构都会让简单平面模型产生系统误差。
- 同一帧可能存在多个有效观测。若只选一个目标更新，剩余装甲板、灯条、R 标或靶盘信息会被浪费。

因此，本项目采用“先维护整体状态，再由整体状态生成各局部观测预测”的方式。检测器负责给出图像观测；跟踪器负责根据结构模型预测这些观测应该出现在图像中的位置；滤波器再用真实观测残差反向修正整体状态。

## 自动瞄准：整车状态估计

自动瞄准状态估计位于 `src/tasks/auto_aim/armor_track/`。核心思想是：同一辆机器人上的多块装甲板不是彼此独立的目标，而是同一刚体上的多个观测面。系统维护车体中心、速度、姿态、角速度和装甲板几何参数，再由这些状态生成每块装甲板、每条灯条在图像中的预测。

### 状态定义

状态定义在 `src/tasks/auto_aim/armor_track/motion_model.hpp`：

```text
x = [cx, vcx, cy, vcy, cz, vcz, rot_z, vyaw, log_r1, log_r2, h, rot_y, rot_x]^T
```

各分量含义如下：

| 分量 | 含义 |
| --- | --- |
| `cx, cy, cz` | 整车中心在 `odom` 系下的位置 |
| `vcx, vcy, vcz` | 整车中心速度 |
| `rot_x, rot_y, rot_z` | 整车姿态的 SO(3) 旋转向量 |
| `vyaw` | 绕车体系 z 轴的角速度 |
| `log_r1, log_r2` | 装甲板到车体中心的半径，使用对数形式保证半径为正 |
| `h` | 四装甲目标中另一组装甲板的高度差 |

普通四装甲目标中，偶数编号装甲板使用 `log_r1` 且高度差为 0，奇数编号装甲板使用 `log_r2` 和高度差 `h`。前哨站复用 `log_r1/log_r2/h` 中的后两个槽位作为 `OUTPOST01DZ/OUTPOST02DZ`，并把半径约束到固定前哨站半径。基地和前哨站使用退化模型，不估计完整自由车体姿态，只保留主要 yaw 方向。

### 整车与装甲板几何

整车位姿由状态直接生成：

```text
T_car_odom.translation = [cx, cy, cz]^T
R_car_odom = Exp_SO3([rot_x, rot_y, rot_z])
```

第 `i` 块装甲板在车体系下的位置与姿态由结构先验给出：

```text
theta_i = i * 2pi / armor_num
r_i = exp(log_r1) 或 exp(log_r2)
p_i_car = [-r_i cos(theta_i), -r_i sin(theta_i), dz_i]^T
R_armor_car = Rz(theta_i) * Ry(armor_pitch)
T_armor_odom = T_car_odom * T_armor_car
```

这一步把“装甲板跳变”转化为确定的刚体几何关系。无论当前看到的是正面装甲板、侧面装甲板，还是相邻装甲板的一条灯条，它们本质上都是同一个整车状态在不同位置上的投影。只要观测能匹配到对应编号，就可以共同约束同一个状态向量。

### 误差状态注入

自动瞄准跟踪器使用 `ErrorStateEKF`。普通欧氏量直接加法注入，姿态量使用 SO(3) 右乘扰动：

```text
R_car_odom <- R_car_odom * Exp_SO3(delta_rot)
p_car_odom <- p_car_odom + delta_p
delta_rot = Log_SO3(R_nominal^T * R_value)
```

代码中对应两个操作：

```text
inject(delta, x)
box_minus(x_nominal, x_value)
```

`inject` 表示把误差状态注入名义状态，`box_minus` 表示从两个名义状态反推出误差状态。二者保持互逆关系：

```text
x_value = inject(delta, x_nominal)
delta   = box_minus(x_nominal, x_value)
```

右乘姿态扰动的作用，是让姿态误差作用在目标自身的切空间内，避免把旋转向量当作普通三维欧氏量直接相减。采用该误差定义后，预测误差传播和观测更新都在同一误差坐标中线性化，协方差、残差和注入操作保持一致。

### 预测模型

预测阶段采用常速度平移和绕车体系 z 轴旋转模型：

```text
cx <- cx + vcx * dt
cy <- cy + vcy * dt
cz <- cz + vcz * dt
R  <- R * Exp_SO3([0, 0, vyaw * dt])
```

前哨站目标会结合方向投票器 `Voter`，在方向明确后使用固定角速度 `OUTPOST_WZ` 推进；基地目标约束 `vyaw = 0`。

预测传播时，滤波器先扰动上一时刻名义状态，再分别预测，并用 `box_minus` 把两个预测结果的差转换回当前误差坐标：

```text
x_pert      = inject(delta_i, x_prev)
x_pred      = f(x_prev)
x_pert_pred = f(x_pert)
F_i         = box_minus(x_pred, x_pert_pred) / eps
```

多观测更新时，滤波器同样对误差状态做数值线性化，得到与当前注入方式一致的观测雅可比 `H`。这种方式计算量高于直接对欧氏状态求差，但避免了姿态注入和欧氏雅可比混用造成的不一致。

### 过程噪声

过程噪声由 `ArmorTarget::process_noise()` 构造。平移部分以车体系加速度噪声配置，再旋转到 `odom` 系：

```text
Q_accel_car  = diag(q_xyz)
Q_accel_odom = R_car_odom * Q_accel_car * R_car_odom^T
Q_p_p = 1/4 dt^4 Q_accel_odom
Q_p_v = 1/2 dt^3 Q_accel_odom
Q_v_p = 1/2 dt^3 Q_accel_odom
Q_v_v = dt^2 Q_accel_odom
```

姿态误差同样位于车体系切空间。模型只显式估计 `vyaw`，因此 yaw 角加速度噪声按常角加速度模型填入 `rot_z / vyaw` 块：

```text
Q_rot_z_rot_z += 1/4 dt^4 q_yaw
Q_rot_z_vyaw  += 1/2 dt^3 q_yaw
Q_vyaw_vyaw   += dt^2 q_yaw
```

`q_wpr` 用于吸收非 yaw 姿态漂移，例如车体 roll/pitch 小幅误差、地面坡度和外参残差。半径和高度也有独立随机游走噪声；半径状态使用 `log_r`，因此噪声会按当前半径做尺度换算。

### 图像观测模型

自动瞄准观测模型直接工作在图像平面。对某块装甲板或某条灯条，算法根据当前状态生成三维位姿，再通过相机模型投影到图像坐标：

```text
z_hat = project(camera, T_camera_odom^-1 * T_armor_i(x) * P)
residual = z_observed - z_hat
```

主要观测类型包括：

- `UVLMeasure`：单条灯条观测，使用角度、中心点和长度。
- `DiffMeasure`：左右灯条中心深度差观测，用于单完整装甲板场景下补充姿态约束。

灯条观测定义为：

```text
UVL = [angle, center_x, center_y, length]^T
```

其中 `angle` 残差会做 `+-pi` 归一化。完整装甲板在更新时会拆成左右两条灯条观测；单独灯条在未被完整装甲板占用时也可以参与更新。滤波器最小化的不是 PnP 后的三维位姿误差，而是更接近传感器原始测量的图像重投影几何残差。

### 单装甲板深度差约束

当一帧中只有一块完整装甲板完成匹配时，单靠这块装甲板拆出的两条 UVL 观测容易退化。图像中的中心、长度和角度可以约束投影形状，但对“左灯条和右灯条谁更靠近相机”并不总是敏感，整车旋转可能被上一时刻预测牵引。

为补上这个约束，`ArmorTarget::update()` 在 `matched_armors.size() == 1` 且 `armor_pnp()` 成功时，会额外构造一维 `DiffMeasure`：

```text
depth_diff = z(left_light_center_in_camera) - z(right_light_center_in_camera)
```

该观测不把 IPPE 的完整三维位姿写进滤波器，只取左右灯条中心点深度差这一维几何信息。预测观测同样由当前整车状态生成，滤波器使用二者残差更新状态。这样可以在单装甲板场景下给车体姿态增加独立约束，抑制共面 PnP 和纯图像重投影在斜视角下的歧义，同时避免过度相信 IPPE 的绝对位置和姿态。

### 多观测更新

一帧中的所有有效观测会被组织成 `update_multi` 的观测列表：

```text
z = [z_1, z_2, ..., z_n]^T
h(x) = [h_1(x), h_2(x), ..., h_n(x)]^T
residual = z - h(x)
```

扩展卡尔曼滤波器在当前状态附近对 `h(x)` 线性化，得到观测雅可比 `H`，再根据观测噪声 `R` 和状态协方差 `P` 计算卡尔曼增益：

```text
K = P H^T (H P H^T + R)^-1
delta_x = K (z - h(x))
x <- inject(delta_x, x)
```

这里的 `delta_x` 是误差状态，不是普通状态增量。姿态部分通过 `Exp_SO3` 右乘注入，位置、速度、半径和高度等欧氏量继续使用加法注入。

当同一帧加入更多有效观测时，`H^T R^-1 H` 提供的信息量增加，后验状态不确定性下降。因此，同时利用装甲板和灯条不是简单堆叠特征点，而是在滤波框架内增加对整车状态的独立约束。

### 匹配与门控

为避免错误观测进入滤波器，跟踪器在更新前进行显式匹配和门控。

完整装甲板匹配时，算法先根据预测装甲板在相机坐标系下的法向可见性选出候选编号，再把候选装甲板左右灯条端点投影成四点轮廓，与检测四点计算代价。代价主要包含：

- 中心误差
- 边角度误差
- 周长比例误差

通过门限的候选会进入 `greedy_match`。当前实现中 IPPE PnP 主要用于初始化和单完整装甲板的深度差观测，不再作为常规编号匹配的核心代价。

单独灯条匹配时，算法根据预测端点检查长度比例、倾角和位置门限，并以端点距离作为匹配代价。只有通过门控的装甲板和灯条才会进入 `update_multi`。

## 能量机关状态估计

能量机关估计位于 `src/tasks/auto_buff/rune_track/`。与自动瞄准类似，能量机关也利用结构约束：R 标、扇叶和靶盘不是独立目标，而是同一个能量机关平面上的固定结构。系统维护能量机关中心、朝向、旋转相位和大符运动参数，并把检测到的 R 标、扇叶、靶盘统一投影到图像平面更新。

### 状态定义

状态定义在 `src/tasks/auto_buff/rune_track/motion_model.hpp`：

```text
x = [cx, cy, cz, yaw, roll, v_roll, tau, a, w]^T
```

各分量含义如下：

| 分量 | 含义 |
| --- | --- |
| `cx, cy, cz` | 能量机关中心位置 |
| `yaw` | 能量机关平面朝向 |
| `roll` | 当前旋转相位 |
| `v_roll` | 方向未确定时使用的相位速度 |
| `tau` | 大符正弦模型时间参数 |
| `a, w` | 大符角速度模型参数 |

角度状态 `yaw` 和 `roll` 使用角度归一化注入：

```text
yaw  <- normalize_angle(yaw + delta_yaw)
roll <- normalize_angle(roll + delta_roll)
```

### 预测模型

能量机关通过 `Voter` 判断旋转方向，并区分小符和大符模式。

小符模式使用固定角速度：

```text
delta_theta_abs = SMALL_SPEED * dt
```

大符模式使用参数化角速度积分：

```text
b = AMPLITUDE_SUM - a
delta_theta_abs =
    (a / w) * (cos(w * tau0) - cos(w * tau1)) + b * dt
```

当方向仍处于 `Collecting` 状态时，预测使用 `v_roll * dt`；方向确定后，根据顺时针或逆时针给 `delta_theta_abs` 加符号。大符参数 `a` 和 `w` 会被约束在规则允许范围内。

### 观测模型

能量机关主要观测包括：

- `RMeasure`：R 标中心二维像素观测。
- `FanBladeMeasure`：扇叶关键点观测。
- `FanTargetMeasure`：靶盘角点和中心点观测。
- `YPDMeasure`：由 PnP 整理出的 yaw、pitch、distance、yaw/roll 观测，用于扇叶和靶盘编号匹配。

扇叶姿态由状态和编号生成：

```text
roll_i = roll + i * 2pi / FAN_NUM
T_rune_odom = pose(cx, cy, cz, yaw, roll_i)
```

靶盘中心相对扇叶平面有固定偏移：

```text
T_fan_target_odom = T_rune_odom * T_fan_target_rune
```

因此，R 标、扇叶和靶盘观测都可以写成统一形式：

```text
z_hat = project(camera, T_camera_odom^-1 * T_part_i(x) * P)
residual = z_observed - z_hat
```

这使能量机关在只有部分结构可见时仍能更新同一个整体状态。例如检测到带 R 扇叶时，扇叶关键点和内部 R 点可共同约束；检测到靶盘和独立 R 标时，也可以通过中心关系完成初始化和更新。

### 匹配与方向投票

能量机关有 5 个重复扇叶结构，检测结果必须先分配到具体编号。匹配流程大致为：

1. 对候选扇叶或靶盘做 PnP，得到粗略 `YPD` 观测。
2. 用当前预测状态生成 5 个编号的预测观测。
3. 计算马氏距离并用 `match_gate` 门控。
4. 对通过门控的候选执行贪心匹配。

更新后，系统根据 `roll` 的连续变化更新 `Voter`。当方向证据累计到阈值后，`Voter` 从 `Collecting` 转为 `Clockwise` 或 `Counterclockwise`。大符模式则通过多扇叶或多靶盘观测累计证据，满足条件后切换到 `Big`。

### 过程噪声

能量机关过程噪声由 `RuneTarget::process_noise()` 构造：

```text
Q_cx_cx   = dt * q_x
Q_cy_cy   = dt * q_y
Q_cz_cz   = dt * q_z
Q_yaw_yaw = dt * q_yaw
```

`roll / v_roll` 使用常角加速度噪声：

```text
Q_roll_roll   = 1/4 dt^4 q_roll
Q_roll_vroll  = 1/2 dt^3 q_roll
Q_vroll_vroll = dt^2 q_roll
```

大符参数 `a`、`w`、`tau` 使用独立随机游走噪声，用于在规则模型附近缓慢修正参数。

## 与传统 PnP 后滤波的区别

本项目的状态估计重点不在“每帧先求一个目标位姿，再对位姿做滤波”，而在“由整体状态生成多种观测预测，再用图像观测反向修正整体状态”。

这种设计带来几个收益：

- 多观测利用率更高。完整装甲板、单灯条、R 标、扇叶和靶盘都能成为同一状态的约束。
- 减少 PnP 抖动影响。常规更新更依赖图像重投影残差，而不是直接相信单帧 PnP 的绝对位姿。
- 遮挡鲁棒性更好。局部结构可见时仍可更新整体状态，短时缺失时由预测模型维持连续性。
