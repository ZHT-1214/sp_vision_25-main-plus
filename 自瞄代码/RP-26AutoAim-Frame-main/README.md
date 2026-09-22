# RP-26AutoAim-Frame

`RP-26AutoAim-Frame` 是一个用于学习和扩展的插件式自瞄框架示例。  

核心框架代码位于 `src/core`，提供插件接口、动态加载和 `Context` 通信网络。

`src/app_plugin` 保留原有插件名称和通信链路，但仅实现为不包含算法的最小可运行示例。

## 0.设计初衷
我们在 2026 赛季区域赛后设计了这个框架，起点是复盘比赛期间多人协作时遇到的版本管理和模块耦合问题：算法、设备驱动与通信逻辑彼此依赖，修改一个模块往往需要同步修改整套工程，也很难保证不同开发者使用的是一致的算法版本。

因此，我们将稳定的运行时、插件接口和 `Context` 通信网络抽取为核心框架，把具体功能拆分为可以独立编译、加载和替换的插件。这样既能让成员并行开发、快速验证和回滚功能，也能减少算法实现对主程序的影响，同时通过约定好的接口，让各个兵种的算法模块能进行快速的复用。从而使得后续开发更关注功能本身，而不是反复处理工程组织问题。  

我们没有直接采用 ROS，主要是因为项目更关注比赛场景下的轻量化、低耦合和可控性。ROS 提供了完善的节点、通信和工具链，但对当前项目而言，引入完整的 ROS 体系会增加工程管理的复杂度，并且在通信上导致自瞄链路的延迟有一定的上升。


## 1. 框架内容

### 1.1 core 框架装配

项目直接编译 `src/core` 中的核心框架代码：

- 插件接口和通用数据类型。
- `Context` 通信网络。
- 动态库加载和插件工厂。
- 运行时入口 `app`。
- 通用算法和基础工具库。

core 的使用约定和父项目 CMake 设计见 [src/core/README.md](src/core/README.md)。为了便于开源，我们将原本core子项目也作为框架的一部分开源。但在实际开发过程中建议保留其作为子项目引入。

### 1.2 插件动态加载

插件位于 `src/app_plugin`。每个插件编译为动态库，通过 `REGISTER_PLUGIN(...)` 注册，并由运行时按 `config/runtime_config.json` 动态加载。

`app` 不直接包含具体插件头文件，也不直接链接具体插件库。插件能否参与运行由配置决定。

### 1.3 统一输出目录

顶层 CMake 将可执行文件、动态库和静态库统一输出到根目录 `output/`。

运行入口：

```bash
./output/app
```

运行时读取固定配置：

```text
config/runtime_config.json
```

## 2. 目录结构

```text
26-Auto-aim/
├── src/
│   ├── core/                 # 核心框架
│   └── app_plugin/           # 保留原名称的最小插件示例
├── config/
│   └── runtime_config.json   # 插件加载和线程组配置
├── output/                   # 构建产物输出目录
├── build/                    # CMake 构建目录
└── CMakeLists.txt            # 父项目构建装配入口
```

## 3. 插件配置

### 3.1 runtime_config.json

`config/runtime_config.json` 决定默认运行时加载哪些插件、加载顺序和线程组归属。

当前配置：

```json
{
    "plugins": [
        {
            "name": "ReceiveDecoder",
            "library": "libreceive_decoder_plugin_lib.so"
        },
        {
            "name": "CameraManager",
            "library": "libcamera_manager_plugin_lib.so"
        },
        {
            "name": "Detector",
            "library": "libdetector_plugin_lib.so"
        },
        {
            "name": "TrackerManager",
            "library": "libtracker_manager_plugin_lib.so"
        },
        {
            "name": "FireControlSystem",
            "library": "libfire_control_system_plugin_lib.so"
        },
        {
            "name": "SendEncoder",
            "library": "libsend_encoder_plugin_lib.so",
            "AttachTo": "FireControlSystem"
        }
    ]
}
```

字段说明：

- `name`：插件注册名，必须和插件源码中的 `REGISTER_PLUGIN(...)` 保持一致。
- `library`：插件动态库文件名，默认从 `output/` 目录加载。
- `AttachTo`：可选字段，用于把当前插件附加到前面某个插件所属线程组。

### 3.2 AttachTo 线程组

没有 `AttachTo` 时，插件创建独立线程组。

存在 `AttachTo` 时，当前插件加入目标插件所属线程组。目标插件必须已经在 `plugins.json` 前面出现过；如果目标插件本身已经附加到其他插件，当前插件会加入最终所属线程组。

当前配置形成的线程组是：

- `ReceiveDecoder`
- `CameraManager`
- `Detector`
- `TrackerManager`
- `FireControlSystem + SendEncoder`

组内按配置顺序循环调用 `process(context)`。

### 3.3 process 约定

`process()` 表示一次调度调用。它可以等待一次驱动输入，但不能在内部写永久循环；完成一轮处理后应返回。

示例：

```cpp
void MinimalDetector::process(const app::Context &context)
{
    // 通信端点应缓存，避免每次调度都重复查找 Context。
    // 如果同一个插件类会实例化多个对象，应改用实例成员变量而不是局部 static。
    static auto input = context.get_buffer_subscriber<InputFrame>(this);
    static auto output = context.get_buffer_publisher<InputFrameWithNNResults>(this);

    InputFrameWithNNResults result;
    result.input_frame = input.wait_pop();
    output.push(std::move(result));
}
```

## 4. 插件开发

### 4.1 插件职责

插件实现业务逻辑，不负责声明系统级调度规则。输入输出契约优先写在 core 接口层的 `declare()` 中，插件实现只在 `process()` 中通过`context`获取并处理数据。当前公开插件仅用于演示这些机制。

### 4.2 新增插件步骤

1. 在 `src/app_plugin/<module_name>` 下创建插件目录。
2. 继承 `src/core/app/interface` 中已有接口。
3. 实现 `process(const app::Context &context)`。
4. 在插件 `.cpp` 中使用 `REGISTER_PLUGIN("Name", Type)` 注册。
5. 在插件 `CMakeLists.txt` 中生成 `SHARED` 动态库。
6. 链接 `app_interface_lib` 和 `class_loader_lib`。
7. 在 `config/runtime_config.json` 中添加插件 `name` 和 `library`。

### 4.3 CMake 示例

```cmake
set(DETECTOR_PLUGIN_TARGET detector_plugin_lib)

add_library(${DETECTOR_PLUGIN_TARGET} SHARED src/MinimalDetector.cpp)

target_include_directories(${DETECTOR_PLUGIN_TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${DETECTOR_PLUGIN_TARGET} PUBLIC app_interface_lib class_loader_lib)
```

插件库不需要链接到 `app`。运行时会根据 `runtime_config.json` 加载动态库。

## 5. 额外依赖库

运行环境要求：

- Ubuntu 22.04 或更高版本；
- 支持 C++20 的编译器（GCC 11+ 或 Clang 14+）；
- CMake 3.22 或更高版本。

项目构建需要以下额外库：

- OpenCV（4.8版本及以上）；
- Eigen3；
- Sophus；
- Google glog。

Foxglove SDK 已包含在 `src/core/utility/foxglove` 中，无需额外安装。

## 6. 构建运行

```bash
git clone git@github.com:SZURPVision/RP-26AutoAim-Frame.git
cd RP26-AutoAim-Frame
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
./output/app
```

## 7. 常见问题

### 插件未注册或加载失败

- 确认插件源码包含 `REGISTER_PLUGIN("Name", Type)`；
- 确认 `runtime_config.json` 中的 `name` 与注册名完全一致；
- 确认 `library` 与 `output/` 中生成的 `.so` 文件名一致。

`app` 不直接链接具体插件符号；插件由运行时按配置动态加载。

## 8. 许可证

本项目自有代码采用 [MIT License](LICENSE)。仓库中包含的第三方代码和依赖遵循其各自的许可证。
