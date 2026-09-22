# sp_vision_25-main-plus 改动说明与调试指南

> 本文档是对主 `readme.md` 的补充，记录 plus 版本相对原版的所有改动，以及各调试程序的使用步骤。
> 涉及的核心文件：`tasks/auto_aim/`（自瞄算法）、`tools/extended_kalman_filter.*`（EKF）、`src/*_debug*.cpp`（调试程序）、`configs/*.yaml`（参数）。

---

## 一、改动总览

| 序号 | 改动 | 作用 | 涉及文件 |
|------|------|------|----------|
| 1 | EKF 卡方检验（NIS/NEES）修复 | 修正滤波器发散检测的数值错误 | `tools/extended_kalman_filter.*`、`tasks/auto_buff/buff_target.cpp` |
| 2 | 目标关联：马氏距离门控 | 把观测装甲板与 EKF 预测装甲板做关联，剔除野值 | `target.*`、`tracker.*`、`configs/standard3.yaml` |
| 3 | PnP 双解消歧 | 解决平面目标 IPPE 解的二义性，选对朝向 | `solver.cpp` |
| 4 | 录制功能（`--record`） | 录制图像+IMU 四元数，用于回放复现 | `src/auto_aim_debug_mpc.cpp` |
| 5 | 离线 MPC 测试程序 | 视频回放验证轨迹规划器 | `tests/auto_aim_mpc_test.cpp`、`configs/demo_mpc.yaml` |
| 6 | 调试输出清理 | 删除实时循环里的 `std::cout` 刷屏 | `tasks/auto_aim/planner/planner.cpp` |
| 7 | 其他一些其他小问题 | 敌我颜色、射击容差、云台加速度等 | `configs/standard3.yaml` |

---

## 二、改动详细说明

### 1. EKF 卡方检验（NIS/NEES）修复

**问题**：原滤波器的卡方检验存在几处错误——残差向量越界访问、失败标志位“粘滞”无法复位、误用后验协方差代替先验协方差、阈值在多次调用间不更新。

**修复**（`tools/extended_kalman_filter.cpp`）：
- `update()` 先保存先验状态 `x_prior` / `P_prior`；
- 新息 `innovation = z - h(x_prior)`，新息协方差 `S = H·P_prior·Hᵀ + R`；
- `nis = innovationᵀ·S⁻¹·innovation`（新息归一化平方）；
- `nees = (x - x_prior)ᵀ·P_prior⁻¹·(x - x_prior)`（估计误差归一化平方）；
- 新增 `nis_threshold_override` 可选阈值，允许不同维度观测用不同卡方阈值。

---

### 2. 目标关联：马氏距离门控

**背景**：多目标/多装甲板场景下，原代码只按「兵种 + 类型」粗匹配，容易把别的机器人的观测混进当前目标，导致状态跳变。

**做法**（`target.cpp` 的 `gate_mahalanobis()`）：
- 枚举目标所有装甲板 id，用 EKF 预测该装甲板的 yaw-pitch-distance（ypd）几何量；
- 计算观测装甲板与预测装甲板的马氏距离（在 [yaw, pitch, distance, armor_yaw] 四维空间，yaw 做 `limit_rad` 角度包裹）；
- 协方差 `S = H·P·Hᵀ` 加上观测噪声对角阵 R（见下方参数）；
- 返回最小马氏距离 `d²` 和对应的 `matched_id`。

`update()` 中：若 `matched_id < 0` 或 `d² > gate_threshold`，该观测当作野值丢弃、不参与滤波。

**参数**（在 `configs/standard3.yaml` 的 tracker 段）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `gate_threshold` | 13.28 | 门控阈值 = χ²(4自由度) 分位，99% |
| `gate_yaw_noise` | 0.004 | yaw 观测噪声方差 rad²（≈3.6° std） |
| `gate_pitch_noise` | 0.004 | pitch 观测噪声方差 rad² |
| `gate_distance_noise_ratio` | 0.05 | 距离噪声 std = 比例 × 距离（5%） |
| `gate_angle_noise` | 0.09 | 装甲板 yaw 噪声方差 rad² |
步兵相机已经调好，可以不做更改，后续沿用其他相机时，可先使用观察后再调

调参方法见本文档「四、参数调优」。

---

### 3. PnP 双解消歧

**问题**：装甲板是平面目标，`SOLVEPNP_IPPE` 会给出两个镜像解（法向翻转），选错会导致目标姿态来回翻转、坐标跳变。

**做法**（`solver.cpp` 的 `solve()`）：
- 改用 `cv::solvePnPGeneric(..., SOLVEPNP_IPPE)` 得到两个解 `rvecs/tvecs`；
- 依次剔除：
  1. 相机后方解：`tvec[i].z <= 0`；
  2. 背面朝向解：装甲板法向与相机方向点积 `>= 0`（即法向背对相机）；
- 对剩余解用 `cv::projectPoints` 计算重投影误差，取误差最小者。

---

### 4. 录制功能（`--record`）

`auto_aim_debug_mpc` 新增录制开关，运行同时把「图像 + 云台四元数 + 时间戳」写盘，事后可离线回放复现问题（类似 rosbag）。

- 输出目录：`./records/`（以当前时间戳命名 `YYYY-MM-DD_HH-MM-SS.avi` + `.txt`）；
- 帧率：默认 30fps（`tools/Recorder` 构造参数）。

---

### 5. 离线 MPC 测试程序

新增 `tests/auto_aim_mpc_test.cpp`，读取录制好的 `.avi` + `.txt`，跑完整识别→跟踪→MPC 规划流程，窗口可视化「预测装甲板（绿）vs MPC 瞄准装甲板（红）」，并把 yaw/pitch 等数据写进 PlotJuggler 曲线。

配套配置 `configs/demo_mpc.yaml`（含马氏距离门控参数）。

---

### 6. 调试输出清理

`planner.cpp` 的 `plan()` 里删除了每周期打印的 `std::cout << "fire!!!" ...`，避免在 ~100Hz 实时循环里刷屏、拖慢规划。

---


---

## 三、调试代码使用步骤

### 3.1 编译

```bash
cmake -B build
make -C build/ -j`nproc`
```

### 3.2 实车 / 实时调试

#### `auto_aim_debug_mpc`（自瞄 + MPC 规划器，带可视化与录制）

```bash
# 用默认配置 configs/standard3.yaml 运行
./build/auto_aim_debug_mpc configs/demo_mpc

# 指定配置 + 开启录制
./build/auto_aim_debug_mpc configs//demo_mpc.yaml --record=true
```

- 位置参数 `@config-path`：yaml 配置路径（默认 `configs/standard3.yaml`）；
- `--record=true`：开启录制，输出到 `./records/`；
- 窗口显示：绿点 = EKF 预测的装甲板，红点 = MPC 实际瞄准的装甲板；按 `q` 退出。



### 3.3 离线回放测试（读录制的 avi + txt）

录制文件位于 `./records/`，格式为 `<时间戳>.avi` + `<时间戳>.txt`。

#### `auto_aim_mpc_test`（MPC 轨迹规划离线回放）

```bash
# 默认读 assets/demo/demo.avi + demo.txt
./build/auto_aim_mpc_test

# 指定录制文件（不带扩展名）与配置
./build/auto_aim_mpc_test -c=configs/demo_mpc.yaml records/20

```

参数：`-c/--config-path`（默认 `configs/demo_mpc.yaml`）、`-s/--start-index`（起始帧）、`-e/--end-index`（结束帧，0=到结尾）、位置参数（avi+txt 路径前缀）。

#### `auto_aim_test`（普通自瞄离线回放）

```bash
./build/auto_aim_test -c=configs/demo.yaml assets/demo/demo
```

参数同 `auto_aim_mpc_test`。


### 3.5 用 PlotJuggler 看曲线

1. 安装：`snap install plotjuggler`（或源码编译）；
2. 'plotjuggler'启动，在 PlotJuggler 界面中：
找到左侧的 Streaming 区域。
在下拉框中选择 UDP Server。
点击旁边的启动按钮，通常是 Start、播放三角形或齿轮按钮。
在弹出的 UDP Server Settings 中设置：Port: 9870
Message Protocol: JSON

3. 运行主程序后，PlotJuggler 里打开 UDP 数据源即可实时查看 `gimbal_yaw` / `plan_yaw` / `fire` / `target_w` 等曲线，
用于判断实际云台、目标角度和 MPC 输出是否一致：
gimbal_yaw
target_yaw
plan_yaw
用于观察俯仰弹道补偿与跟随误差
gimbal_pitch
target_pitch
plan_pitch
用于观察 EKF 角速度和中心高度是否稳定
w
target_z
target_vz
以及其他观测形式
---

## 四、参数调优（马氏距离门控）
步兵相机已经调好，可以不做更改，后续沿用其他相机时，可先使用观察后再调
**主旋钮：`gate_threshold`（只调它就够了）**

- 调大（如 16.3）→ 门控宽松，接受更多观测。适合「检测/PnP 噪声大、该跟却不跟」的场景；代价是可能混入野值、跟错目标。
- 调小（如 9.5）→ 门控严格，拒野值更强。适合「想防止跟错/抖动」；代价是太严会拒掉正常观测，目标停止更新（表现为锁定后突然松手）。
- 建议范围 9.5（95%）~ 16.3（99.9%），默认 13.28（99%）。

**怎么判断调没调对**：开 debug 日志看 `[Target] gated out, mahalanobis=X` 出现的频率和 X 值——
- X 经常在 13~16 附近、且目标是对的 → 门控偏紧，调大一点；
- 几乎从不出现、但目标老跟错 → 门控偏松，调小一点。

**噪声模型 4 个值**（一般设一次，换相机 / PnP 精度明显变化才动）：

| 参数 | 什么时候调 |
|------|-----------|
| `gate_yaw_noise` / `gate_pitch_noise` | PnP 的角度抖动变大时调大 |
| `gate_distance_noise_ratio` | 远距离目标老被拒时调大（如 0.08） |
| `gate_angle_noise` | 一般不用动 |
蓝框稳定，绿色其他三块上下动：EKF 外推或 r/l/h 模型问题
蓝框和绿色当前观测框一起动：检测/PnP/外参问题
绿色稳定，只有红框动：MPC 预测或 decision_speed/delay_time 问题
红框、绿框都稳定，但瞄准仍偏：yaw/pitch offset 或云台控制问题
> 说明：门控是在 ypda 几何空间做的预筛，与 EKF 实际用的 8 维重投影观测是两套噪声；这 4 个值只影响「这个观测要不要收」，不影响 EKF 内部的滤波结果。

参考文档：
吉林大学TARS Go战队https://github.com/Fskaaaaaaaa/jlu_vision_26 
浙江大学Hello World战队 https://github.com/IC-Alan/HWauto_buff2026 
深圳大学-RobotPilots战队  https://github.com/SZURPVision/RP-26AutoAim-Frame 
南京理工大学 Alliance 战队 https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2 
给目标、规划和发弹建立统一时间戳，补偿完整系统延迟。
重新设计 PnP 双解时序消歧。
保证每帧只进行正确的一次观测关联和 EKF 更新。
增加严格开火门控，目标过期、PnP 无效、MPC 失败时绝不 fire。
增加 EKF、弹道和 MPC 的 finite、边界和 solver 状态保护。
统一 Gimbal 串口发送线程。
