# ORB-SLAM3 语义 Mask 接入修改文档

更新时间：2026-04-07

## 1. 修改目标

将 YOLO 分割节点输出的语义 Mask（8UC1，0=动态，255=静态）从外部接口传入 ORB-SLAM3，并在 ORB 特征提取阶段真正使用该 Mask，过滤动态区域特征点。

## 2. 本次代码改动清单

### 2.1 接口链路打通（System -> Tracking -> Frame -> ORBextractor）

1. `System::TrackRGBD` 新增可选参数 `semanticMask`
- 文件：`include/System.h`
- 文件：`src/System.cc`
- 行为：
  - 允许调用方在 RGB-D 跟踪时传入语义 Mask。
  - 若 Mask 尺寸与输入图像不一致，内部会用 `INTER_NEAREST` 对齐到当前输入尺寸。

2. `Tracking::GrabImageRGBD` 新增可选参数 `semanticMask`
- 文件：`include/Tracking.h`
- 文件：`src/Tracking.cc`
- 行为：
  - 将语义 Mask 传递给 `Frame` 的 RGB-D 构造函数。

3. `Frame` 的 RGB-D 构造函数新增 `semanticMask` 参数
- 文件：`include/Frame.h`
- 文件：`src/Frame.cc`
- 行为：
  - RGB-D 构造阶段调用 `ExtractORB` 时，传入 Mask。

4. `Frame::ExtractORB` 新增可选 `mask` 参数
- 文件：`include/Frame.h`
- 文件：`src/Frame.cc`
- 行为：
  - 左目（RGB-D 主路径）提取 ORB 时，将 Mask 传递给 `ORBextractor::operator()`。

### 2.2 ORBextractor 中启用 Mask 过滤

5. `ORBextractor::operator()` 启用 `_mask` 逻辑
- 文件：`src/ORBextractor.cc`
- 文件：`include/ORBextractor.h`（注释更新）
- 具体逻辑：
  - 接收 `_mask` 后统一转换为单通道 `CV_8UC1`。
  - 若尺寸不匹配，按输入图像大小使用最近邻缩放。
  - 在每层图像金字塔上，对 keypoint 做像素级过滤：
    - `mask(y, x) > 0` 保留
    - `mask(y, x) == 0` 删除
  - 之后再计算 descriptor 与输出，保证最终特征只来自静态区域。

## 3. 兼容性说明

1. 所有新增参数均为“默认可选参数”
- 不传 Mask 时，行为保持原版 ORB-SLAM3 逻辑。

2. 对现有 RGB-D 示例代码无强制改动
- 旧调用仍可编译运行。

## 4. 外部调用建议（ROS2 异步语义节点）

推荐在 ROS2 封装层做异步缓存：

1. RGB-D 主回调只做同步图像取帧。
2. 语义回调只更新“最新 Mask 缓冲”。
3. 每次调用 TrackRGBD 时，从缓冲取最新 Mask（可为空）。

示例（伪代码）：

```cpp
cv::Mat latest_mask = semantic_buffer.GetLatest();  // 可能为空
slam_system.TrackRGBD(rgb, depth, stamp, imu_vec, "", latest_mask);
```

## 5. 验收建议

1. 无 Mask：轨迹与原版一致。
2. 有 Mask：动态人群区域特征点明显减少。
3. 高动态场景：地图拖影和动态噪点下降。
4. 性能：检查跟踪线程耗时，确认未引入阻塞等待。

## 6. 后续可选优化

1. 在 ORBextractor 里引入“每层 Mask 预缓存”，减少每帧 resize 次数。
2. 对 Mask 做形态学膨胀，扩大动态物体边缘保护带。
3. 支持类别级策略（如只过滤 person/vehicle）。
