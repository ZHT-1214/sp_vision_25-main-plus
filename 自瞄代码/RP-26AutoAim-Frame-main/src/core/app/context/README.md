# app::Context

`app::Context` 是插件系统的数据通信“接线板”。它的唯一职责是：**基于各插件显式声明的数据输入和输出，在运行时建立安全的 Latest-Only 数据连接。**

`Context` **不**负责线程调度，**不**加载动态库，也**不**解析配置文件。这些由 `runtime` 和 `class_loader` 负责。

---

## 1. 核心工作机制

`Context` 的工作分为两个阶段：

1. **组网检查阶段 (`check_io_network`)**
   运行时在启动阶段会调用所有注册插件的 `declare(context)`。`Context` 会汇总这些声明，检查类型、方向、Topic 是否匹配。如果匹配，就为这些通信节点分配底层的通信工具（`LatestBuffer` 或 `LatestChannel`）。
2. **运行阶段 (`process`)**
   插件在每次调度循环中，通过 `context.get_*` 从已经建立好的网络中获取数据发布者（Publisher）或订阅者（Subscriber），进行数据的收发。

---

## 2. 怎么声明与收发数据？

所有插件接口都应继承 `app::PluginBase`，并实现 `declare` 和 `process` 方法。

### 2.1 声明契约 (`declare`)
在 `declare()` 中，插件必须且仅需说明自己“需要什么”以及“产出什么”。

```cpp
void Detector::declare(app::Context &context) override
{
    // 声明需要消费一帧图像（作为 Input）
    context.declare_input_buffer<InputFrame>(this);
    // 声明会产出检测结果（作为 Output）
    context.declare_output_buffer<InputFrameWithNNResults>(this);
}
```

### 2.2 收发数据 (`process`)
在 `process()` 中，插件通过与之对应的 `get_*` 方法获取操作端点，处理完成后必须立即返回（不能内部写死循环）。

```cpp
void MinimalDetector::process(const app::Context &context) override {
    // 1. 获取并缓存通信端点
    // 强烈建议使用 static 来缓存端点，避免每次循环都去 Context 中查找的开销。
    // 特别是 LatestChannel 的 Subscriber 带有内部状态（用于判断是否有新数据），必须被复用！
    //（注：如果你的同一个插件类会在配置中被实例化多次，请务必将它们改为类的成员变量而不是局部 static）
    static auto input = context.get_buffer_subscriber<InputFrame>(this);
    static auto output = context.get_buffer_publisher<InputFrameWithNNResults>(this);

    // 2. 消费数据 (wait_pop 通常用于获取最新数据)
    InputFrame frame = input.wait_pop();

    // 3. 执行业务逻辑...
    InputFrameWithNNResults result;

    // 4. 发布结果
    output.push(std::move(result));
}
```

---

## 3. Buffer vs Channel：该选哪个？

`Context` 提供两种**只保留最新数据 (Latest-Only)** 的底层工具，绝不会缓存历史堆积数据。

| 通信工具 | 适用场景 | 规则约束 | 获取语法对应 |
|---|---|---|---|
| **`LatestBuffer<T>`** | **一对一的数据流水线** (如 Camera -> Detector)。下游消费后数据即被清空。 | 同一组 Buffer 最多只能有 **1 个** 消费者 | `declare_input_buffer`<br>`get_buffer_subscriber` |
| **`LatestChannel<T>`**| **一对多的状态广播** (如 全局底盘状态)。多方可独立、重复读取最新状态。 | 同一组 Channel 最多只能有 **1 个** 发布者 | `declare_input_channel`<br>`get_channel_subscriber` |

> 💡 **为什么有上述规则约束？**
> Buffer 只允许 1 个消费者，Channel 只允许 1 个发布者。这些规定并非人为刻意刁难，而是由其底层锁机制与极简无锁并发特质所决定的。如果强行违背，极易引发死锁或数据时序跳变。详情请深入参阅 [底层传输模型的设计考量](../../utility/threads/docs/数据传输模型.md)。

---

## 4. 话题区分 (Topic)

当你的系统中存在**相同数据类型的多条独立链路**时（例如系统中有两个相机，它们都会输出 `InputFrame`），就需要用到 `topic` 字符串来区分它们。

* **不传 Topic（默认）**：如果某种数据类型在系统中只有一条链路，直接省略即可，底层会使用默认的空字符串。
  ```cpp
  context.declare_output_buffer<InputFrame>(this);
  ```
* **显式指定 Topic**：如果有多条链路，必须在声明和获取时传入唯一的名称。
  ```cpp
  context.declare_output_buffer<InputFrame>(this, "front_camera/frame");
  context.declare_output_buffer<InputFrame>(this, "rear_camera/frame");
  ```

**注意**：同一条链路的发布者和订阅者必须保持 **类型、通信工具 (Buffer/Channel) 和 Topic** 三者完全一致。此外，`Topic` 更像是一个全局链路名称，请**不要**把同一个 `Topic` 字符串复用给不同的数据类型，否则 `Context` 在组网时会报错。

---

## 5. 常见错误与检查规则

`Context` 在组网检查或端点获取时，会非常严格地校验你的行为，不满足条件将直接抛出异常。

### 🚨 组网期异常 (发生在启动阶段)
* `[Context] duplicate plugin node`：同一个插件对象被重复添加到了 Context 中。
* `LatestBuffer can have at most one consumer`：你试图让多个插件去 Input 订阅同一个 Buffer。
* `LatestChannel can have at most one producer`：你试图让多个插件去 Output 发布同一个 Channel。
* `declarations are incompatible`：你试图在同一个 Topic 下声明不同类型的数据，或同时混用 Buffer 和 Channel。

### 🚨 运行期异常 (发生在 `process` 阶段)
抛出 `not registered for plugin` 通常是因为你在获取时出错了：
1. `get_*` 时指定的类型或 Topic 与 `declare` 时不一致。
2. 声明了 Buffer，获取时却用了 Channel 的 API。
3. 声明了 Input，获取时却试图拿 Publisher。
4. 压根忘记在 `declare` 里声明。

### 💡 最佳实践建议
* **绝不要私下通信**：只在 `declare` 中建立依赖，不要让插件之间传递裸指针。
* **按需缓存端点**：如果你需要长期持有一个端点（特别是 `LatestChannel<T>::Subscriber`，它带有记录是否更新过的内部状态），建议在 `process` 的第一次调用时将其保存为插件类的成员变量，避免每次循环都去 `Context` 中反复创建获取。
* **孤立链路警告**：如果有链路只有输出没有输入，或者只有输入没有输出，`Context` 会报出 warning。虽然程序还能跑，但你应该检查这是否符合你的设计预期。
