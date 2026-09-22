# 【RM-2026能量机关算法开源】浙江大学Hello World战队

RoboMaster 能量机关（Buff / 大符 / 小符）自动瞄准算法库。

从 Hello-World-Vision/autoaim2026
中抽离而来的纯 C++ 实现，**不依赖 ROS2**，只依赖 `Eigen3` 与 `OpenCV`（推理后端见下文），
方便集成到任意机器人软件框架中。

## 功能范围

覆盖能量机关自瞄链路中的模型检测与检测结果后处理：

- **detector**（目标检测）：加载 TensorRT 序列化模型 `buff.engine`，完成 letterbox、推理、NMS 和 9 关键点解码
- **selector**（目标筛选）：从多个检测到的扇叶中选出应该瞄准的目标
- **locate**（位姿解算）：对扇叶 4 个角点做平面 PnP 解算，处理镜像歧义的双解选择
- **tracker**（状态跟踪）：
  - 小符：固定角速度模型 + 卡尔曼滤波跟踪 roll/yaw
  - 大符：基于 RANSAC 的正弦转速模型拟合 + 卡尔曼滤波跟踪 yaw
  - 通用的跟踪状态机（LOST / CONVERGING / TRACKING / TEMP_LOST）
- **predictor**（位置预测）：根据跟踪到的运动模型，外推目标未来某一时刻的位置



## 算法概述

整个自瞄链路的数据流：

```
BGR图像
  → Detector  （letterbox → TensorRT 推理 → 解码 + NMS + 9 关键点）
  → Selector  （颜色/激活筛选 → PnP 求相机夹角 → 选夹角最小的扇叶）
  → PnP       （IPPE 平面四点解算 + 双解歧义消解 → 相机系位姿 Pose3f）
  → Tracker   （状态机 LOST/CONVERGING/TRACKING/TEMP_LOST → 运动模型跟踪）
  → Predictor （按运动模型外推 → 输出未来打击点坐标）
```

### Detector 检测

- 输入 `buff.onnx`（由 `trtexec` 转成 `buff.engine`），输出形状 `[1, 5040, 26]`：
  每行是 `[cx, cy, w, h, 4 类得分, 9×(x, y)]`。
- 类别映射：`b_inactive / b_active / r_inactive / r_active`（扇叶未激活/激活 × 蓝/红）。
- 流程：letterbox 等比缩放填充 → TensorRT 推理 → 逐候选按类别最大得分过滤（`confidence_threshold`）
  → 同类框做 NMS（`nms_threshold`）→ 关键点坐标还原到原图。
- 9 个关键点中 0~7 是打击板八边形顶点，取相邻点中点生成 PnP 用的
  `Top/Left/Bottom/Right` 四点；第 8 点是 R 中心，不参与单个扇叶 PnP。

### Selector 目标筛选

- 只保留"我方要打的那一色 + 未激活"的扇叶（蓝队打红符，红队打蓝符）。
- 对每个候选用 IPPE 解一次位姿，按"相机光轴与扇叶中心的夹角"从小到大排序，
  选夹角最小的一个作为瞄准目标，避免打在靠近画面边缘的扇叶上。

### PnP 位姿解算

- 用 `cv::solvePnPGeneric(..., SOLVEPNP_IPPE)` 对平面四点求两个镜像候选解。
- 双解消歧：首帧选重投影误差更小的解；后续帧比较两个解与上一帧位姿的
  "平移差 × 2 + 旋转角差"加权距离，选更连续的解，抑制解跳变。
- 输出统一到相机系（前 x / 左 y / 上 z），可直接送入 tracker。

### Tracker 跟踪

- **跟踪状态机**：`LOST → CONVERGING → TRACKING`，跟丢时 `TRACKING → TEMP_LOST`
  （超 `max_temp_lost_frames` 帧回 LOST）。大符 CONVERGING 超 `max_converging_frames`
  帧兜底进 TRACKING。
- **小符**：转速固定，用两个一维卡尔曼滤波（`KF<1>`）分别跟踪 roll 和 yaw，
  R 中心用 EMA 平滑；roll 接近 ±90° 奇异区时强制置位滤波值。
- **大符**：转速服从正弦 `v(t) = A·sin(ωt+φ) + C`。由相邻帧 roll 差分求瞬时速度，
  过滤跳变/超速点后送入 RANSAC 正弦拟合；内点数达到 `min_inliers` 即认为拟合成功，
  用模型平滑预测转速。RANSAC 参数见 `config/tracker/tracker.yaml`。

  **RANSAC 拟合细节**（实现见 `src/tracker/ransac_filter.cpp`）：

  模型要解的参数是 `(A, ω, φ, C)`，但 ω 出现在三角函数内部，无法直接线性求解。
  做法是**固定 ω 再对剩下的线性参数做最小二乘**，而 ω 在合理区间内随机采样：

  1. 数据源：每帧用 `roll_velocity = (roll_t - roll_{t-1}) / Δt` 求瞬时速度，
     丢弃跳变（角差 > `switch_buff_angle`）、速度超过 `max_abs_speed`（2.09 rad/s）、
     以及 roll 奇异处的点后，存入 `(相对时间 t, 速度 v)` 序列。
  2. 若两个采样点时间间隔超过 5s，视为目标丢失/重开，清空数据重新开始。
  3. 开始拟合前先保证数据量 ≥ 3 个点。
  4. 迭代 `max_iterations = 200` 次，每次：
     - 从 `[min_omega, max_omega]`（即 1.884 ~ 2.0 rad/s）按均匀分布**随机抽一个 ω**；
     - 从数据中**随机取 3 个点**，构造线性方程组
       `[sin(ωtᵢ), cos(ωtᵢ), 1] · [A₁, A₂, C]ᵀ = vᵢ`，用 SVD 最小二乘解得 `A₁, A₂, C`；
     - 换算回极坐标 `A = √(A₁²+A₂²)`，`φ = atan2(A₂, A₁)`；
     - 若 `A` 不在 `[min_amplitude, max_amplitude]`（0.78 ~ 1.045 rad/s）范围则丢弃该次候选；
     - 用该候选模型对**全部数据**统计内点数：`|vᵢ − (A·sin(ωtᵢ+φ) + C)| < threshold`（0.5 rad/s）；
     - 内点数超过当前最优则更新最优参数。
  5. 迭代完成后，若最优模型的内点数 ≥ `min_inliers = 100`，`ransac_ready()` 返回真，
     tracker 用该正弦模型平滑预测 roll 速度，并从 CONVERGING 进入 TRACKING。
  6. 数据量超过 300 个点时弹出最旧的点，避免时间窗过长、模型漂移。

  注：ω 是连续值，无法穷举，代码用的是 `[min_omega, max_omega]` 区间内
  200 次均匀随机采样（每次配 3 个随机样本点），200 次中内点数最多的那组即最终模型。

### Predictor 位置预测

- 根据 tracker 输出的运动状态（小符的 `roll_velocity`，或大符的正弦参数 `A/ω/φ/C`）
  把 roll 外推到"当前时刻 + 延迟 + 弹丸飞行时间"，再用
  `R 中心 + 半径绕 R 中心旋转` 求出打击点在相机系下的预测坐标。

## 依赖

- C++20
- Eigen3
- OpenCV (使用了 `cv::FileStorage` 读取参数、`cv::solvePnP`/`solvePnPGeneric` 做 PnP 解算)
- TensorRT + CUDA（`buff.engine` 推理）

## 目录结构

```
include/buff_algo/
  common/       # 公共数据类型 (BuffDetection/BuffState/Pose3f)、数学工具、EMA滤波器
  detector/     # buff.engine 推理、NMS 与关键点解码
  tracker/      # 卡尔曼滤波器、RANSAC正弦拟合、跟踪状态机、BuffTracker
  predictor/    # BuffPredictor 位置外推
  locate/       # BuffPnPSolver 位姿解算
  selector/     # BuffSelector 目标筛选
src/            # 对应的实现文件
config/         # 示例参数文件（YAML，cv::FileStorage 格式）
config/detector/detector.yaml   # 检测阈值 / 模型路径 / 大小符与颜色
config/tracker/tracker.yaml     # 跟踪器参数
config/predictor/predictor.yaml # 预测器参数
examples/       # 独立可运行的示例程序
```

## 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./buff_algo_example
```

常用 CMake 选项：

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `BUFF_ALGO_BUILD_EXAMPLES` | `ON` | 编译示例程序 |
| `TENSORRT_ROOT` | 自动查找 | TensorRT 安装根目录 |
| `CUDA_TOOLKIT_ROOT_DIR` | 自动查找 | CUDA 安装根目录 |

TensorRT + CUDA 是编译必需的，`cmake` 会自动探测；找不到时报错，可用上面的
`-D` 选项指定安装位置。

也可以直接把本目录作为子目录用 `add_subdirectory` 集成到你自己的 CMake 项目中，
链接 `buff_algorithm` 这个 target 即可。

## ONNX 转 TensorRT engine

本库只做 `*.engine` 的推理，需要先用 `trtexec` 把训练好的 `buff.onnx` 转成
TensorRT 序列化模型：

```bash
/usr/src/tensorrt/bin/trtexec \
    --onnx=buff.onnx \
    --saveEngine=buff.engine \
    --fp16 \
    --verbose
```

- `--fp16`：以半精度构建，速度更快（要求 GPU 支持 FP16）
- `--verbose`：输出详细日志，便于排查算子不支持等问题
- 生成 `buff.engine` 后即可直接交给 `buff_algo_example` 使用

## 快速上手

核心数据流：

```
cv::Mat (BGR图像)
    -> BuffDetector::detect / detect_with_keypoints  // TensorRT 推理并输出 BuffDetection（可选关键点）
    -> BuffSelector::select_buffs      // 筛选出应该打的那个目标
    -> BuffPnPSolver::solve_pnp        // 解算出相机坐标系下的位姿 Pose3f
    -> Buff(pose)                      // 构造观测量（换算出 R 字中心、roll角等）
    -> BuffTracker::push / update      // 跟踪滤波，得到 BuffState
    -> BuffPredictor::set_state / predict_position  // 预测提前量位置
```

`buff.onnx`（经 `trtexec` 转为 `buff.engine`）的类别映射为 `b_inactive/b_active/r_inactive/r_active`。
模型输出的前 8 个关键点沿打击板边缘排列，解码器取相邻点对的中点，生成 PnP 所需的
`Top/Left/Bottom/Right` 四点；第 9 点为 R 中心，不进入单个扇叶的 PnP。

视频检测示例：

```bash
# 默认：处理当前目录 buff.mp4，模型 buff.engine（TensorRT），输出 buff_detected.mp4
./build/buff_algo_example
```

模型加载 `*.engine`（TensorRT 序列化模型，见上文转换方法）。运行时会显示实时画面
（按 `q`/`Esc` 退出）、把 9 个关键点以彩色圆点编号画到画面上，并在终端用
`print_colored_status_info()` 打印 tracker 的状态（大/小符、转速、yaw 等）。

示例程序的检测参数不硬编码，而是从 `config/detector/detector.yaml` 读取：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `buff_model_name` | `buff.engine` | TensorRT 模型路径 |
| `buff_confidence_threshold` | `0.5` | 检测置信度阈值 |
| `buff_nms_threshold` | `0.4` | NMS IoU 阈值 |
| `buff_mode` | `1` | 能量机关类型：`1`=小符，`2`=大符 |
| `buff_color` | `1` | 要打的能量机关颜色：蓝队打红符传 `1`，红队打蓝符传 `0` |

其中 `buff_mode` 为大小符切换。模型输出只有 `b_*/r_*` 类别（扇叶 vs R 中心），
本身无法区分大小符，所以需要在 config 里显式配置：小符走固定角速度模型，
大符走 RANSAC 正弦转速模型。命令行传入的 `engine`/`confidence` 会覆盖 config 中的同名参数。

具体用法可参考 `examples/example.cpp`：读入视频后，完整串起
「检测 → 筛选 → PnP → 跟踪 → 预测」链路，并演示如何只用 `detect_with_keypoints()`
拿到关键点做可视化。

注意：`BuffPnPSolver`/`BuffSelector` 只解算相机坐标系下的位姿，
如果你的机器人云台/底盘会转动，需要在外部把相机坐标系下的位姿变换到
一个相对静止的参考系（例如底盘 yaw 轴坐标系）后再送入 `BuffTracker`，
否则跟踪器无法正确区分"目标运动"和"自身云台运动"。这部分坐标变换
与具体机器人平台强相关，不包含在本库中。

## 参考文献

【RM2025-自瞄算法开源】同济大学SuperPower战队(https://bbs.robomaster.com/article/803315)
(https://github.com/TongjiSuperPower/sp_vision_25/)
## 许可证

MIT License，见 [LICENSE](./LICENSE)。

本项目还包含第三方库 [termcolor](https://github.com/ikalnytskyi/termcolor)（BSD 3-Clause），
详见 [THIRD_PARTY_NOTICES.md](./THIRD_PARTY_NOTICES.md)。
