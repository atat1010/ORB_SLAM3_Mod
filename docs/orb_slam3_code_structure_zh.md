# ORB-SLAM3 代码结构说明（面向语义改造）

## 1. 顶层模块

1. `System`
- 入口调度层。
- 对外提供 `TrackMonocular / TrackStereo / TrackRGBD`。
- 管理 Tracking、LocalMapping、LoopClosing、Viewer 线程生命周期。

2. `Tracking`
- 前端主线程。
- 输入图像后构造 `Frame`，执行特征提取、匹配、位姿估计、重定位判定。
- 将关键帧插入请求发送给 LocalMapping。

3. `LocalMapping`
- 局部建图线程。
- 处理关键帧插入、MapPoint 维护、局部 BA。

4. `LoopClosing`
- 回环检测与全局一致性优化线程。
- 执行回环约束、地图融合、全局图优化。

5. `Viewer` / `FrameDrawer` / `MapDrawer`
- 可视化模块。

## 2. 语义改造相关主链路

1. 外部调用入口
- `System::TrackRGBD(...)`

2. 前端入口
- `Tracking::GrabImageRGBD(...)`

3. 帧构造与特征提取
- `Frame::Frame(RGBD...)`
- `Frame::ExtractORB(...)`

4. 特征点最终生成
- `ORBextractor::operator(...)`

这条链路决定了“Mask 是否真正影响特征提取”。

## 3. 目录分工

1. `include/`
- 头文件声明。
- 对外接口、类结构、成员定义。

2. `src/`
- 核心实现。
- 推荐把算法行为改动放在这里，并保持 `include/` 与 `src/` 同步。

3. `Examples/` 与 `Examples_old/`
- 数据集与示例运行入口。
- 在工程接入 ROS2 时，通常只借鉴输入管线，不建议把复杂业务逻辑塞入示例文件。

4. `Thirdparty/`
- 第三方依赖（DBoW2、g2o、Sophus）。
- 非必要不要改。

## 4. 关键数据对象

1. `Frame`
- 单帧图像的特征、描述子、深度、位姿容器。
- 语义改造后，Mask 通过它传入特征提取。

2. `MapPoint`
- 三维点。

3. `KeyFrame`
- 关键帧。

4. `Atlas/Map`
- 多地图管理与地图容器。

## 5. 语义方案推荐落点

1. 异步通信与缓存
- 放在 ROS2 封装层（节点层），不要放在 ORB 核心类中。

2. 动态点过滤
- 放在 `ORBextractor::operator()` 或 Frame 构造后的 keypoint 后处理。
- 当前实现采用前者，语义更直接，链路更清晰。

3. 性能优化
- 优先在节点层做（TensorRT、消息 QoS、缓存策略）。
- ORB 内核保持“只消费 mask，不等待 mask”。

## 6. 常见改造陷阱

1. 在 Tracking/Frame 内部直接做 YOLO 推理
- 会阻塞前端线程，导致掉帧与跟踪抖动。

2. 修改接口但忘记全链路联动
- System、Tracking、Frame、ORBextractor 任一处漏改都会导致 Mask 不生效。

3. mask 尺寸或类型不统一
- 必须保证单通道、与输入图像同尺寸（或在入口统一 resize）。

4. 过滤逻辑方向写反
- 当前约定：0=动态（剔除），255=静态（保留）。
