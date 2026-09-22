# 并发推理

本文描述 AWAKENING 中并发推理链路的实现。相关代码主要位于：

- `src/utils/scheduler/`
- `src/utils/buffer.hpp`
- `src/utils/net_detector/openvino/`
- `src/utils/net_detector/tensorrt/`
- `src/runtime/standard.cpp`
- `src/runtime/sentry.cpp`
- `src/runtime/radar_detect.cpp`

项目中的并发推理不是单一位置的 `std::thread`，而是由三层机制共同组成：

1. 运行时调度器把图像采集、检测、跟踪、控制等任务拆成节点，并允许不同帧的任务并行推进。
2. 检测任务使用 `std::counting_semaphore` 控制同时进入推理后端的帧数，避免 GPU/CPU 推理资源被过度占用。
3. OpenVINO 和 TensorRT 后端内部使用 `ResourcePool` 维护多个可复用推理资源，使多个检测调用可以安全地并发执行。

整体目标是在保证输出顺序和实时性的前提下，提高相机高帧率输入下的吞吐量。

## 总体数据流

以 `standard` 运行时为例，主链路可以简化为：

```text
camera source
    -> CommonFrame
    -> detector
    -> OrderedQueue
    -> tracker
    -> aimer / controller
```

相机线程不断产生带有递增 `id` 的 `CommonFrame`。检测任务可能因为推理耗时不同而乱序完成，因此结果会先进入 `OrderedQueue`，再按帧号有序输出给跟踪器。

并发推理链路可以表示为：

```text
Frame_i
  -> detector task
  -> semaphore.try_acquire()
  -> NetDetector::detect()
  -> backend resource pool
  -> post process
  -> OrderedQueue
  -> tracker
```

如果当前推理并发数已达到配置上限，检测任务会跳过该帧的推理，并仍然输出一个带有帧号的空结果。这样可以避免检测节点阻塞相机输入和后续调度。

## 调度器：帧级并行

项目的运行时任务由 `Scheduler` 管理，代码位于 `src/utils/scheduler/`。

`Scheduler` 的核心概念是：

- `SourceNode`：数据源，例如相机图像输入。
- `TaskNode`：处理节点，例如检测、跟踪、控制。
- `IOPair<Tag, Type>`：用类型标记输入输出通道。

节点通过输入输出 tag 自动连接。运行时每当源节点产生数据，调度器会把下游任务放入队列，并通过 `tbb::task_group` 执行：

```text
source.execute()
    -> schedule(detector)
        -> detector.execute()
            -> schedule(tracker)
```

由于调度器不会等待上一帧完整处理结束才接收下一帧，因此当相机帧率高于单帧检测速度时，多个检测任务可能在时间上重叠。这就是项目需要并发推理资源池和信号量限流的原因。

## 信号量限流

检测节点中使用 `std::counting_semaphore` 控制最大并发推理数量。自动瞄准、能量机关和雷达识别都有类似逻辑。

自动瞄准中对应配置为：

```text
config["armor_detector"]["max_infer_num"]
```

能量机关中对应配置为：

```text
config["rune_detector"]["max_infer_num"]
```

雷达识别中对应配置为：

```text
config["max_infer_num"]
```

检测任务执行时会尝试获取信号量：

```cpp
bool got = detector_sem->try_acquire();
utils::SemaphoreGuard guard(*detector_sem, got);

if (got) {
    result = detector.detect(frame, focus);
}
```

这里使用 `try_acquire()` 而不是阻塞式 `acquire()`，意味着当推理资源已满时，当前帧会被主动跳过检测，而不是阻塞等待。这样做的取舍是：

- 优先保证实时性和低延迟。
- 允许极端负载下丢弃部分检测帧。
- 保持后续跟踪器可以继续按时间推进。

`SemaphoreGuard` 负责 RAII 释放信号量，避免检测过程提前返回时忘记释放并发额度。

## 有序输出队列

并发检测会带来一个问题：后进入推理的帧可能先完成，检测结果会乱序。跟踪器通常要求按时间顺序更新，因此项目使用 `utils::OrderedQueue` 对结果重排。

`OrderedQueue` 要求元素带有 `id` 字段。入队时：

```text
if item.id == current_id:
    push to main_queue
    current_id++
    flush buffered consecutive ids
else if item.id > current_id:
    store in buffer
else:
    discard old item
```

出队时，`dequeue_batch()` 会一次取出当前已经连续到达的结果批次：

```text
detector result -> OrderedQueue -> ordered batch -> tracker
```

因此，即使推理任务并发执行，跟踪器看到的仍然是按帧号递增的检测结果。对于被信号量跳过的帧，运行时仍会入队空结果，保证帧号连续推进，避免后续结果长期堵在 buffer 中。

## 后端资源池

OpenVINO 和 TensorRT 后端都继承自 `NetDetectorBase`：

```cpp
class NetDetectorBase {
public:
    struct OutPut {
        cv::Mat output;
        Eigen::Matrix3f transform_matrix;
        cv::Mat resized_img;
    };

    virtual OutPut detect(const cv::Mat& img, PixelFormat format) = 0;
};
```

并发安全的关键是：一次 `detect()` 调用不能与其它调用共享不可重入的推理上下文、输入输出 buffer 或 CUDA stream。项目通过 `ResourcePool<T>` 解决这个问题。

### ResourcePool

`ResourcePool<T>` 位于 `src/utils/buffer.hpp`。它维护一组资源，每个资源带有一个原子 `busy` 标记：

```text
resources = [
    { value: T, busy: false },
    { value: T, busy: false },
    ...
]
```

调用 `acquire()` 时，资源池会扫描所有资源，并通过 CAS 抢占空闲项：

```text
for resource in resources:
    if busy.compare_exchange(false -> true):
        return Handle(resource)
return null
```

返回的 `Handle` 是 RAII 对象，析构时自动把 `busy` 置回 `false`。这使推理后端可以用非阻塞方式获取独占上下文：

```cpp
auto r = pool.acquire();
if (!r) {
    return {};
}
auto& ctx = *r;
```

资源池不负责等待。如果没有空闲资源，调用者会立即失败或走备用路径。这与运行时信号量共同保证了系统不会因为过载而无限排队。

## OpenVINO 并发推理

OpenVINO 后端实现位于 `src/utils/net_detector/openvino/net_detector_openvino.cpp`。

### InferRequest 池

OpenVINO 的 `CompiledModel` 可以创建多个 `ov::InferRequest`。每个请求对象保存一次推理所需的内部状态，因此并发调用时不能让多个线程共用同一个 `InferRequest`。

初始化时，后端根据配置创建请求池：

```text
infer_request_buffer_num = config["infer_request_buffer_num"]
```

```cpp
for (int i = 0; i < infer_request_buffer_num; ++i) {
    infer_request_buffer_.add_resource(create_infer_request());
}
```

每次推理时：

```cpp
auto r = infer_request_buffer_.acquire();
if (!r) {
    return infer(input_tensor);   // 临时创建请求作为兜底
}
return infer(input_tensor, *r);
```

因此 OpenVINO 的并发能力由两部分决定：

- 运行时信号量允许同时进入检测的帧数。
- `infer_request_buffer_num` 允许同时复用的 `InferRequest` 数量。

### 输入预处理

OpenVINO 后端在 `detect()` 中执行：

1. 根据输入格式检查是否需要重新初始化模型预处理。
2. 使用 `utils::letterbox()` 将输入图像缩放到目标尺寸。
3. 构造 `ov::Tensor`，指向 `resized_img.data`。
4. 使用独占 `InferRequest` 推理。
5. 将输出 tensor clone 成 `cv::Mat`。

流程为：

```text
img
  -> letterbox
  -> ov::Tensor
  -> InferRequest.infer()
  -> output cv::Mat
```

OpenVINO 的预处理管线中配置了输入布局、颜色格式、类型转换、尺度缩放和目标颜色格式转换，减少 C++ 侧手写预处理逻辑。

### 格式变化与重初始化

OpenVINO 后端记录当前输入格式 `input_format_`。如果新帧格式发生变化，会重新构建预处理并重新编译模型：

```cpp
if (format != input_format_) {
    input_format_ = format;
    init();
}
```

`resetting_` 是原子标志。重置期间 `detect()` 会直接返回空结果，避免并发线程使用处于重建中的模型对象。

## TensorRT 并发推理

TensorRT 后端实现位于 `src/utils/net_detector/tensorrt/net_detector_tensorrt.cpp`。

TensorRT 的并发推理核心是多个独立的 `Ctx`：

```cpp
struct Ctx {
    std::shared_ptr<nvinfer1::IExecutionContext> context;
    std::array<void*, 2> device_buffers;
    std::vector<float> output_buffer;
    cudaStream_t stream;
    __cuda::LetterBox::Ptr letter_box;
};
```

每个 `Ctx` 包含：

- 独立的 `IExecutionContext`
- 独立输入/输出 GPU buffer
- 独立输出 CPU buffer
- 独立 CUDA stream
- 可选 CUDA letterbox 预处理对象

这些对象组合成一次推理的完整上下文。多个并发调用只要拿到不同 `Ctx`，就不会互相覆盖输入输出 buffer，也不会共用 TensorRT execution context。

### Context 池

初始化时，根据配置创建多个上下文：

```text
copy_context_num = config["copy_context_num"]
```

创建每个上下文时，会检查 GPU 剩余显存比例：

```text
free_mem_ratio = free_mem / total_mem
if free_mem_ratio < min_free_mem_ratio:
    stop creating more contexts
```

因此实际 context 数可能小于 `copy_context_num`。这可以避免为了提高并发而耗尽显存。

### 推理流程

TensorRT `detect()` 的流程为：

```text
img
  -> optional CUDA letterbox preprocess
  -> acquire Ctx
  -> setTensorAddress(input/output)
  -> enqueueV3(stream)
  -> cudaMemcpyAsync(output D2H)
  -> cudaStreamSynchronize(stream)
  -> output cv::Mat
```

如果启用 CUDA 预处理：

```cpp
tensor = ctx.letter_box->letterbox_pitched(..., ctx.stream)
ctx.device_buffers[INPUT_IDX] = tensor
```

否则使用 CPU `letterbox + blobFromImage`，再异步拷贝到 GPU：

```cpp
cudaMemcpyAsync(input_device, blob.ptr<float>(), ..., ctx.stream)
```

推理使用：

```cpp
ctx.context->enqueueV3(ctx.stream)
```

输出使用异步拷贝回 host buffer，并在当前 stream 上同步：

```cpp
cudaMemcpyAsync(ctx.output_buffer.data(), output_device, ..., ctx.stream)
cudaStreamSynchronize(ctx.stream)
```

同步点保证 `detect()` 返回时 `cv::Mat` 中的数据已经可用。虽然单次 `detect()` 对调用者表现为同步函数，但多个 `detect()` 调用可在不同 `Ctx` 和不同 CUDA stream 上并发执行。

## 运行时示例：自动瞄准与能量机关

在 `standard.cpp` 中，检测节点会根据当前模式选择自动瞄准或能量机关检测。

自动瞄准：

```text
max_infer_num = armor_detector.max_infer_num
try_acquire semaphore
    -> ArmorDetector::detect(frame, net_focus, detect_light)
enqueue Armors by frame id
dequeue ordered batch
```

能量机关：

```text
max_infer_num = rune_detector.max_infer_num
try_acquire semaphore
    -> RuneDetector::detect(frame, net_focus, enemy_color)
enqueue RuneDetection by frame id
dequeue ordered batch
```

两条链路都使用：

- `counting_semaphore` 限制运行时并发数量。
- 后端资源池保证同一模型内部的推理上下文互不冲突。
- `OrderedQueue` 保证输出按帧号有序。

自动瞄准还会在进入检测前根据上一帧跟踪状态计算 `net_focus` 和 `detect_light`，减少每次推理需要处理的图像区域。

## 运行时示例：雷达识别

雷达识别同样使用信号量限流：

```text
max_infer_num = config["max_infer_num"]
try_acquire semaphore
    -> Detector::detect(frame, full_image_focus)
    -> periodically Detector::detect_armors(frame, outpost_bbox)
enqueue Cars by frame id
dequeue ordered batch
```

`Detector::detect()` 内部包含两次 TensorRT 推理：

1. 车辆检测模型 `car_trt_`
2. 车辆 ROI 拼接图上的装甲板检测模型 `armor_trt_`

这两个模型各自拥有自己的 `NetDetectorTensorrt` 实例和 context 池。运行时信号量限制的是整个雷达检测任务的并发数量，后端 context 池限制的是每个模型内部可同时执行的推理数量。

## 配置关系

并发推理相关配置可以分成三类：

| 层级 | 配置 | 作用 |
| --- | --- | --- |
| 运行时检测节点 | `max_infer_num` | 限制同时进入检测任务的帧数 |
| OpenVINO 后端 | `infer_request_buffer_num` | 预创建并复用多个 `InferRequest` |
| TensorRT 后端 | `copy_context_num` | 预创建多个 execution context / stream / buffer |
| TensorRT 后端 | `min_free_mem_ratio` | 显存不足时停止创建更多 context |
| TensorRT 后端 | `use_cuda_preproces` | 是否使用 CUDA letterbox 预处理 |

通常需要满足：

```text
max_infer_num <= 后端可用上下文数量
```

如果 `max_infer_num` 远大于后端资源池容量，多余的检测调用会在后端获取资源失败并返回空结果，等价于浪费调度开销。如果 `max_infer_num` 太小，则推理吞吐可能无法充分利用硬件。

## 为什么采用非阻塞并发

本项目选择“非阻塞限流 + 有序输出”的策略，而不是把所有帧排队等待推理，原因是视觉控制更关心新鲜数据：

- 旧图像即使最终推理完成，也可能已经不适合控制当前云台。
- 阻塞等待会把延迟从检测节点传播到整条控制链路。
- 高帧率相机下，宁可丢弃部分检测帧，也要避免控制指令使用过时观测。

因此，当推理资源满载时，系统会跳过当前帧检测，让下一帧有机会尽快进入推理。跟踪器则依靠状态预测维持短时间连续性。

## 工程特点

本项目并发推理实现有几个特点：

- 任务级并发由 `Scheduler + tbb::task_group` 提供。
- 运行时并发上限由 `std::counting_semaphore` 控制。
- 后端上下文由 `ResourcePool` 以无阻塞方式独占获取。
- OpenVINO 使用 `InferRequest` 池复用请求对象。
- TensorRT 使用 execution context、CUDA stream、输入输出 buffer 组成的 context 池。
- 输出结果通过 `OrderedQueue` 按帧号重排，保证跟踪器按时间顺序更新。
- 过载时主动跳帧，优先保证实时性而不是完整处理每一帧。
