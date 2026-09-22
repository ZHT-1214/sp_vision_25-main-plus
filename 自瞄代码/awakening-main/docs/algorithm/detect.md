# 识别模块

本文描述 AWAKENING 中自动瞄准、能量机关和雷达识别的检测链路。相关代码主要位于：

- `src/tasks/auto_aim/armor_detect/`
- `src/tasks/auto_buff/rune_detect/`
- `src/tasks/radar_detect/`
- `src/utils/net_detector/`

项目中的识别模块并不只依赖单一神经网络。自动瞄准和能量机关都保留了传统 CV 链路，并可按配置切换 OpenCV、OpenVINO 或 TensorRT 后端；雷达识别面向远距离、大视场场景，主要使用 TensorRT 的车辆/装甲板两级检测。识别结果最终会进入跟踪、状态估计、弹道解算和可视化模块。

## 公共推理封装

神经网络推理统一通过 `utils::NetDetectorBase` 抽象。不同任务只需要给出模型输入尺寸、像素格式和归一化尺度：

```cpp
utils::NetDetectorBase::Config {
    .target_format = ...,
    .preprocess_scale = ...,
    .target_w = ...,
    .target_h = ...,
}
```

后端根据配置实例化为：

- `NetDetectorOpenVINO`
- `NetDetectorTensorrt`

推理输出包含两部分：

- `output`：网络原始输出张量。
- `transform_matrix`：输入图像经过 resize/letterbox 等预处理后的坐标变换矩阵。

各任务的 `Infer` 类负责把网络输出解码成业务对象，并把结果坐标通过 `transform_matrix` 还原回 ROI 图像坐标；随后检测器再加上 ROI 偏移，还原到原图坐标。

## 自动瞄准识别

自动瞄准识别由 `ArmorDetector` 负责，核心文件包括：

- `armor_detector.cpp`
- `armor_detector.hpp`
- `armor_infer.cpp`
- `armor_infer.hpp`

输入为 `CommonFrame`、网络 ROI `net_focus` 和是否启用传统灯条检测的标志。输出为：

```cpp
std::tuple<std::vector<Light>, std::vector<Armor>>
```

其中：

- `Light` 表示传统 CV 或网络结果整理出的单灯条。
- `Armor` 表示完整装甲板，包含编号、颜色、关键点、置信度等信息。

### 网络识别链路

当配置后端不是 `opencv` 时，`ArmorDetector` 会创建 `ArmorInfer` 和对应推理后端。当前 `ArmorInfer` 支持多种模型格式：

- `TUP`
- `RP`
- `AT1`
- `AT2`

不同模型对应不同输入尺寸、颜色格式、归一化方式和输出解码逻辑。后处理主要完成：

1. 从网络输出中解析装甲板候选。
2. 根据置信度阈值过滤。
3. 解码装甲板类别、颜色和关键点。
4. 执行 top-k 与 NMS。
5. 对高度重合且类别一致的候选做关键点合并。

网络检测在 `net_focus` 对应的 ROI 内执行：

```text
roi = src_img(net_focus)
net_output = net_detector.detect(roi)
armors = armor_infer.process(net_output.output)
armor.transform(net_output.transform_matrix)
armor.add_offset(net_focus.tl())
```

这种设计使网络既可以全图识别，也可以在跟踪器给出的局部 ROI 内识别。

### 传统 CV 灯条识别

传统 CV 链路主要用于检测灯条并辅助装甲板构造，核心实现位于 `ArmorDetector::detect_lights()`、`correct_corners()`、`detect_cv()`。它既可以作为纯 OpenCV 后端独立识别装甲板，也可以在网络后端启用时提供单灯条观测，服务后续整车状态估计。

#### 图像预处理与轮廓提取

灯条检测首先在 `detect_light` 指定的 ROI 内运行；若没有传入 ROI，则默认使用整图：

1. 在给定 `bbox` 内截取检测区域。
2. 将 ROI 转为灰度图。
3. 使用 `bin_threshold` 做二值化。
4. 使用 `cv::findContours(..., cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE)` 提取外轮廓。

简化流程为：

```text
detect_roi = src(bbox)
gray = grayscale(detect_roi)
bin = threshold(gray, bin_threshold)
contours = find_external_contours(bin)
```

网络后端启用时，CV 灯条检测的二值阈值可以由网络结果自适应修正。代码会从网络装甲板关键点中取出左右灯条线段，在灰度图上采样平均亮度，得到参考阈值：

```text
threshold = average_brightness(net_lights) - net_ref_threshold_tol
```

这样可以让传统灯条检测跟随当前曝光和网络检测到的真实灯条亮度变化，减少固定阈值在不同场地光照下的失效概率。

#### 灯条几何筛选

每个轮廓会被构造成 `Light` 候选。候选灯条主要通过几何条件过滤：

```text
ratio = light.width / light.length
min_wh_ratio < ratio < max_wh_ratio
light.tilt_angle < max_angle
```

其中：

- `width / length` 用于排除过粗、过扁或面积异常的亮斑。
- `tilt_angle` 用于排除倾角过大的环境灯、反光条或不合法轮廓。
- 轮廓点数量过少时直接丢弃，避免 `minAreaRect` 等几何量不稳定。

通过这些筛选后，候选才会进入颜色判断和角点修正。

#### 灯条颜色判断

颜色判断直接在轮廓点上累计红蓝通道值。根据输入图像格式选择通道索引：

```text
BGR: r_idx = 2, b_idx = 0
RGB: r_idx = 0, b_idx = 2
```

对轮廓内采样点累计：

```text
sum_r = Σ pixel[r_idx]
sum_b = Σ pixel[b_idx]
avg_diff = abs(sum_r - sum_b) / contour_point_count
```

若 `avg_diff <= diff_threshold`，说明红蓝差异不足，候选被丢弃；否则：

```text
sum_r > sum_b -> RED
sum_b > sum_r -> BLUE
```

这一步的作用是把白色高亮、反光、场地灯等颜色不明确的亮斑过滤掉。

#### 灯条端点修正

初始灯条端点来自轮廓几何，但远距离和运动模糊下轮廓边界会抖动。`correct_corners()` 会对灯条上下端点做一次亮度分布修正。

流程如下：

1. 取灯条外接矩形，并按 `ROI_SCALE` 做小幅扩展。
2. 提取灰度 ROI，归一化到固定亮度范围。
3. 根据亮度矩计算质心。
4. 对 ROI 中非零亮度点做 PCA，得到灯条主轴方向。
5. 沿主轴正反方向搜索亮度突变点。
6. 多条平行搜索线得到的候选端点取平均，作为修正后的上下端点。

端点搜索的核心思想是：灯条端点附近会出现由亮到暗的亮度下降，沿主轴方向寻找最大下降点：

```text
diff = gray(prev_point) - gray(cur_point)
corner = argmax(diff), gray(prev_point) > mean_brightness
```

如果搜索失败，则保留原始端点。修正后的端点会保存在 `light.corrected` 中，供后续装甲板配对和状态估计使用。

#### 灯条配对与装甲板构造

纯 CV 后端中，系统会将灯条按 `center.x` 排序，然后枚举左右灯条组合：

```text
for left in lights:
    for right in lights after left:
        if left.color != right.color:
            continue
        armor = Armor(left, right)
        if is_armor(armor):
            keep
```

`Armor(left, right)` 会整理装甲板四个关键点、中心、宽高比例等几何信息。当前 `is_armor()` 主要检查装甲板宽高比例：

```text
min_ratio < armor.cv->ratio < max_ratio
```

通过几何筛选后，装甲板会进入数字分类：

1. 根据左右灯条四点做透视变换。
2. 截取数字 ROI。
3. 二值化。
4. 批量送入数字分类模型。
5. 丢弃编号为 `UNKNOWN` 的候选。

最后，去除被标记为重复的装甲板，输出 `Armor` 列表。

CV 链路的优势是可解释、延迟低，并且能在网络输出不足时提供单灯条观测。单灯条结果不会只用于识别，也会进入后续整车状态估计，作为图像重投影观测的一部分。

### 数字分类与颜色分类

对完整装甲板，检测器还可以启用数字分类和颜色分类：

- 数字分类：通过透视变换把装甲板数字区域 warp 到固定大小，再送入分类模型。
- 颜色分类：根据左右灯条 ROI 的 RGB/BGR 通道均值差判断红蓝。

数字分类用于确认装甲板编号；颜色分类用于过滤非敌方目标或修正网络颜色输出。

## 基于整车状态先验的显式空间注意力

大多数 RoboMaster 视觉算法使用的神经网络模型都采用固定尺寸输入。输入网络前通常需要对工业相机图像进行缩放、填充或裁剪。常见做法是 `letterbox`：在保持原图长宽比的前提下，将图像缩放到网络输入尺寸，再用 padding 补齐空白区域。

该方法能避免几何形变，但也会带来一个问题：工业相机图像的长宽比通常与网络输入长宽比不一致，整幅图经过 `letterbox` 后往往会被整体缩小。远距离装甲板本来只占很少像素，缩放后有效纹理和边缘信息进一步减少，关键点回归、数字分类和置信度都会受到影响。

本项目利用整车状态估计提供的几何先验，在检测阶段引入显式空间注意力机制。不再把整张图像无差别地送入检测器，而是根据当前整车状态、相机外参和时间戳预测目标在当前图像中的可能区域，并构造随跟踪置信度变化的 ROI。这样网络输入关注的是目标高概率出现区域，而不是包含大量背景的整幅图像。

代码中这一逻辑由 `ArmorTarget::get_net_focus_roi()` 提供。其核心流程为：

1. `need_focus()` 判断当前目标是否适合使用聚焦 ROI；若不适合则返回整图。
2. `expanded()` 预测当前时刻所有装甲板左右灯条端点在图像中的位置，并生成包围框。
3. 根据目标类型扩展包围框，普通目标扩展比例较小，基地目标扩展比例更大。
4. 根据网络输入宽高比修正 ROI，减少后续 letterbox padding。
5. 将 ROI 扩为方形区域，适配常见方形网络输入。
6. 根据 `timestamp - last_update` 逐渐扩大 ROI；若目标长时间未更新，则退化为整图搜索。

简化表示为：

```text
predicted_lights = project(all armor lights from target_state)
rect = bounding_rect(predicted_lights)
rect = expand(rect)
rect = fit_network_aspect_ratio(rect)
rect = grow_with_lost_time(rect)
```

对于神经网络识别链路，这种 ROI 机制有两个直接收益：

- ROI 的长宽比可以主动匹配网络输入，减少 padding 和无效背景区域，使输入像素更多用于描述目标本身。
- 当目标距离较远时，ROI 裁剪后再 resize 到网络输入尺寸，相当于对目标区域进行局部放大，保留并增强远距离小目标的灯条边缘、数字区域和角点结构。

对于传统 CV 链路，整车先验同样可以构造更小的搜索区域。传统灯条检测依赖颜色阈值、亮度分割、轮廓提取、形态筛选和灯条配对。搜索区域越大，环境灯光等干扰越容易进入候选集，同时 CPU 开销和误匹配概率也会增加。根据整车状态直接预测装甲板灯条端点并生成紧凑 ROI，可以从源头减少无效候选，降低 CPU 压力，并提升匹配稳定性。

在运行时中，`standard`、`hero`、`sentry` 等入口会根据当前跟踪状态设置 `net_focus` 和传统检测区域：

```text
net_focus    <- ArmorTarget::get_net_focus_roi(...)
detect_light <- ArmorTarget::expanded(...)
ArmorDetector::detect(frame, net_focus, detect_light)
```

这使检测和跟踪形成闭环：检测结果更新整车状态，整车状态又反过来指导下一帧检测区域。

## 能量机关识别

能量机关识别由 `RuneDetector` 负责，核心文件包括：

- `rune_detector.cpp`
- `rune_detector.hpp`
- `rune_infer.cpp`
- `rune_infer.hpp`

输入为 `CommonFrame`、检测 ROI `focus` 和敌方颜色。输出为 `RuneDetection`，主要包含：

- `fan_blades`：带 R 标的扇叶网络检测结果。
- `rune_rs`：传统 CV 检测到的 R 标。
- `fan_targets`：传统 CV 检测到的靶盘。
- `rune_flowing_lights`：流动灯条等辅助结构。

### 网络扇叶识别

当后端不是 `opencv` 时，`RuneDetector` 会创建 `RuneInfer` 和对应推理后端。网络在 `focus` ROI 内推理，`RuneInfer` 对输出进行后处理：

1. 解码扇叶候选和关键点。
2. 根据置信度阈值过滤。
3. 执行 top-k 与 NMS。
4. 对重合候选做关键点合并。
5. 使用 `transform_matrix` 还原到 ROI 坐标。
6. 加上 `focus.tl()` 偏移，还原到原图坐标。

网络输出主要用于提供稳定的扇叶关键点，后续跟踪器会通过 PnP 和多观测更新估计能量机关整体状态。

### 传统 CV 结构识别

能量机关 CV 链路主要负责识别 R 标、靶盘和辅助高亮结构，核心实现位于 `RuneDetector::preprocess()`、`get_rune_rs()`、`get_rune_fan_targets()`。它与网络扇叶检测互补：网络提供扇叶关键点，CV 提供 R 标和靶盘结构，后续跟踪器再把这些观测统一到能量机关状态中。

#### 预处理与颜色过滤

检测首先在 `focus` ROI 内运行：

1. 灰度化或颜色通道处理。
2. 二值化和形态学闭操作。
3. `findContours` 提取轮廓树。
4. 根据颜色过滤非敌方轮廓。

当前实现中，预处理主要使用灰度阈值：

```text
gray = grayscale(roi)
bin = threshold(gray, bin_threshold)
bin = morphology_close(bin, ellipse_kernel_3x3)
contours, hierarchy = findContours(bin, RETR_TREE)
```

随后 `color_filter()` 会根据轮廓外接矩形 ROI 的红蓝通道均值判断颜色。若当前目标颜色不匹配，则将该轮廓标记为 used，后续不再参与 R 标或靶盘识别：

```text
color = mean_color(contour_bbox)
enemy_color mismatch -> used_flags[i] = true
```

这里使用 `RETR_TREE` 而不是只取外轮廓，是因为能量机关图案存在嵌套结构，R 标和靶盘候选都需要利用轮廓层级关系。

#### R 标识别

R 标识别由 `get_rune_rs()` 完成。候选必须满足以下条件：

1. 轮廓未被颜色过滤或其它结构占用。
2. 必须是顶层轮廓，即 `hierarchy[i][3] == -1`。
3. 轮廓面积位于 `[rune_r_min_area, rune_r_max_area]`。
4. 最小外接旋转矩形中心位于 `focus_r` 中。
5. 外接矩形接近正方形。
6. 轮廓面积与外接矩形面积的填充率足够高。

对应的几何约束可以写成：

```text
area_min < contour_area < area_max
ratio = max(w, h) / min(w, h)
ratio - 1.0 <= rune_r_1x1ratio_tol
fill_ratio = contour_area / (w * h)
fill_ratio >= rune_r_fill_ratio_min
```

R 标检测会重点关注 ROI 中心附近区域。代码中会构造一个位于 `focus` 中心的 `focus_r`，用于约束 R 标候选：

```text
focus_r = center_square(focus)
rune_rs = get_rune_rs(contours, focus_r)
```

这可以减少边缘高亮干扰对 R 标识别的影响。检测到的 R 标、靶盘和扇叶都会加上 ROI 偏移，统一回原图坐标。

当某个 R 标候选被接受后，代码会调用 `mark_parent()` 标记其顶层父轮廓，并将当前轮廓标记为 used，避免同一结构被后续靶盘检测重复使用。

#### 靶盘候选点提取

靶盘识别由 `get_rune_fan_targets()` 完成。它不是直接寻找一个完整大轮廓，而是先提取若干小轮廓中心点，再按层级和空间聚类组合成靶盘。

候选点筛选条件为：

```text
rune_pan_min_area < contour_area < rune_pan_max_area
moment.m00 != 0
```

通过筛选后，使用图像矩计算轮廓中心：

```text
center = (m10 / m00, m01 / m00)
```

同时记录该轮廓的顶层父轮廓 `parent_top_id`。随后根据 `parent_top_id` 分组，使同一父结构下的候选点优先组合。

#### 靶盘点聚类

每个父轮廓组内，代码对候选点做基于距离的 BFS 聚类：

```text
if distance(candidate_i, candidate_j) <= rune_pan_cluster_radius:
    same cluster
```

每个聚类至少需要 3 个点才会进入后续判断。聚类后的点集会计算最小外接旋转矩形：

```text
rr = minAreaRect(cluster_points)
ratio = max(rr.width, rr.height) / min(rr.width, rr.height)
```

如果 `ratio > rune_pan_max_square_ratio`，说明点集形状过于细长，不像靶盘角点结构，会被丢弃。

#### 靶盘角点整理

通过聚类形状检查后，代码会按照点到旋转矩形中心的距离排序，选择距离最远的 4 个点作为靶盘角点：

```text
dist = squared_norm(point - rr.center)
corners = top4_by_dist(dist)
```

这一步利用了靶盘角点通常位于结构外侧的特点。最终构造 `RuneFanTarget`：

```text
fan_target.center = rr.center
fan_target.rr = rr
fan_target.corners[0..3] = selected outer points
```

被接受的聚类点对应轮廓会被标记为 used，并同步标记其父轮廓，避免重复输出。

#### 坐标还原与颜色补充

能量机关 CV 检测在 ROI 坐标系中进行，输出前统一加上 `focus.tl()`：

```text
rune_r.add_offset(focus.tl())
fan_target.add_offset(focus.tl())
```

R 标和靶盘还会根据各自外接矩形 ROI 的平均颜色补充 `RuneColor`。这些结果会与网络扇叶结果一起组成 `RuneDetection`，交给后续跟踪器进行 PnP、编号匹配和状态更新。

### 与跟踪器的关系

能量机关检测输出不会直接生成控制指令，而是进入 `RuneTracker`。跟踪器根据 R 标、扇叶和靶盘的观测做匹配、PnP 和 ESEKF 更新。检测阶段只负责尽可能稳定地提供结构化图像观测：

```text
image -> RuneDetector -> RuneDetection -> RuneTracker -> RuneTarget
```

相比自动瞄准，能量机关检测更依赖结构组合：单独的 R 标、扇叶或靶盘都可能不足以稳定控制，但它们组合起来可以共同约束能量机关平面位姿和旋转相位。

## 雷达识别

雷达识别的检测器位于：

- `src/tasks/radar_detect/detector.cpp`
- `src/tasks/radar_detect/detector.hpp`

雷达场景视野更大、目标距离更远，检测目标从“单个装甲板”扩展到“车辆 + 装甲板”。当前实现使用 TensorRT 两级检测：

- `car_trt_`：车辆检测模型。
- `armor_trt_`：装甲板检测模型。

两个模型都使用 RGB 输入、`1/255` 归一化和 `1280 x 1280` 网络输入尺寸。

### 车辆检测

`Detector::detect()` 首先在给定 `focus` ROI 内检测车辆：

```text
roi = src_img(focus)
car_output = car_trt.detect(roi)
cars = post_process_car(car_output.output)
```

车辆后处理流程为：

1. 遍历网络输出列。
2. 根据 `car_confidence_threshold` 过滤低置信度候选。
3. 将中心点宽高形式转换为 `cv::Rect2f`。
4. 使用 `cv::dnn::NMSBoxes` 做 NMS。
5. 通过 `transform_matrix` 将框还原到 ROI 坐标。

车辆框会记录时间戳，并作为装甲板二级检测的候选区域。

### 车内装甲板检测

雷达识别不是直接对整图做装甲板检测，而是先裁剪车辆 ROI，再在车辆内部检测装甲板。为了减少多次推理开销，代码会把多个车辆 ROI 拼接成一张网格图：

```text
car roi 0 ┐
car roi 1 ├── resize to same size -> concatenated_img -> armor_trt.detect()
car roi 2 ┘
```

装甲板后处理流程为：

1. 解码每个候选框和类别得分。
2. 取最大类别作为装甲板编号。
3. 根据 `armor_confidence_threshold` 过滤。
4. 执行 NMS。
5. 通过 `transform_matrix` 还原到拼接图坐标。
6. 判断候选中心落在哪个车辆 ROI 网格中。
7. 将装甲板框映射回原车辆 ROI，再映射回原图。
8. 根据装甲板 ROI 的颜色通道差判断红蓝。

最后每个 `Car` 会持有自己的 `armors` 列表，并调用 `tidy()` 整理结果。

### 直接装甲板检测

除车辆两级检测外，`Detector::detect_armors()` 也支持直接在给定 `focus` ROI 内检测装甲板。该接口适合只需要装甲板结果的场景：

```text
roi = src_img(focus)
armor_output = armor_trt.detect(roi)
armors = armor_post_process(armor_output.output)
```

结果同样会进行坐标还原、颜色判断和 ROI 偏移。

## 三条识别链路对比

| 模块 | 主要目标 | 主要方法 | 输出 |
| --- | --- | --- | --- |
| `auto_aim` | 装甲板、灯条、编号、颜色 | 网络装甲板检测 + CV 灯条检测 + 数字/颜色分类 | `Light`、`Armor` |
| `auto_buff` | R 标、扇叶、靶盘 | 网络扇叶检测 + CV 结构识别 | `RuneDetection` |
| `radar_detect` | 车辆、车内装甲板 | TensorRT 车辆检测 + 车辆 ROI 内装甲板检测 | `Car`、`Armor` |

三者的共同点是都把识别输出整理成结构化观测，再交给后续跟踪和状态估计模块；区别在于场景假设不同：

- 自动瞄准强调高帧率、低延迟和单目标连续跟踪，因此引入整车先验 ROI 来提高小目标检测质量。
- 能量机关强调结构组合和旋转相位估计，因此同时保留网络扇叶和 CV R 标/靶盘检测。
- 雷达识别强调大视场多目标检测，因此采用车辆和装甲板两级检测，并通过车辆 ROI 聚合减少装甲板误检。
