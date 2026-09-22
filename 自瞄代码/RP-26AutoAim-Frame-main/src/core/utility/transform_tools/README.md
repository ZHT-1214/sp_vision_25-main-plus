# transform_tools (坐标变换与数学工具)

`transform_tools` 是基于 `Eigen3`、`Sophus` 和 `OpenCV` 封装的三维空间变换组件。它提供了强类型的角度转换、安全的位姿叠加（`TF`）以及能自动推导变换链路的坐标系树（`TFTree`）。

## 1. 依赖与引入

**CMake Target**: `transform_tools_lib`

```cmake
target_link_libraries(your_plugin PRIVATE transform_tools_lib)
```

**头文件**:
```cpp
#include "transform_tools/transform_tools.h"
#include "transform_tools/frame_id.h"
```

---

## 2. 强类型角度：`Angle`

自瞄解算中通常存在三种角度度量：用于三角函数计算的**弧度制**、用于调参和人眼观察的**角度制**，以及与底层电控通信的**电控量（0~8192）**。
`Angle` 类通过隐藏底层弧度值，提供了一套防止换算混乱和“忘加负号”的强类型安全机制。

### 2.1 构造与字面量
支持通过工厂方法或者 C++11 字面量构建。所有的输入都会被自动归一化到 `[-pi, pi)` 区间：

```cpp
using namespace transform_tools;
using namespace transform_tools::literals;

// 工厂方法
Angle a1 = Angle::from_degree(90.0);
Angle a2 = Angle::from_rad(1.57);
Angle a3 = Angle::from_ECS_degree(4096.0); // 电控量 4096 对应 0度

// 字面量语法糖 (推荐)
Angle a4 = 90_deg;
Angle a5 = 3.14_rad;
```

### 2.2 取值与运算
```cpp
double rad = a1.to_rad();
double deg = a1.to_degree();
double ecs = a1.to_ECS();

// 支持基础运算，运算结果始终保持在 [-pi, pi)
Angle a_sum = 90_deg + 180_deg; // 结果为 -90_deg
Angle a_neg = -a_sum;
```

---

## 3. 旋转分量：`Yaw`, `Pitch`, `Roll`

为了防止在构造旋转矩阵时弄错轴或顺序，本模块使用强类型的 `SingleEulerAngle` 派生类。

```cpp
Yaw yaw(90_deg);
Pitch pitch(15_deg);
Roll roll(0_deg);

// 可以直接转换为 Eigen 的旋转矩阵
Eigen::Matrix3d R = yaw.to_rotation_mat();
```

---

## 4. 坐标系变换 (TF & TFTree)

> **注：** 关于 `TF`（位姿变换容器）和 `TFTree`（自动变换树）的底层数学推导、Craig 约定用法以及详细 API 示例，团队已有足够丰富的外部架构文档。此处不再赘述，请直接查阅相关外部文档。

---

## 5. 类型互转工具
自瞄系统不可避免地要在 `OpenCV` 与 `Eigen` 之间倒腾数据。本模块提供了一系列轻量内联的 `cv2eigen` 和 `eigen2cv` 函数：

```cpp
cv::Point3d cv_pt(1, 2, 3);
Eigen::Vector3d eig_pt = cv2eigen(cv_pt);

Eigen::Matrix3d eig_mat = Eigen::Matrix3d::Identity();
cv::Mat cv_mat = eigen2cv(eig_mat);
```
支持的类型包括：`Point3d`, `Vec3d`, `Point2d`, `Vec2d`, `Mat`, `Matx33d` 等。
