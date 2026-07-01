# Point Cloud Web Viewer

本项目用于在 Web 端查看实时或静态点云，并对仓库内目标物体进行体积计算。当前主要面向两类输入：

- 实时 LiDAR 点云：RS-LiDAR 或 Unitree LiDAR 通过 ROS 2 发布点云，后端通过 WebSocket 推送到网页。
- 静态 LAS 点云：在网页中加载 `.las` 文件，直接在前端显示，并可上传给后端执行 Patchwork++ 地面分割。

## 计算目标

体积计算不是直接对整幅点云求体积，而是按以下思路处理：

1. 获取一段稳定的点云数据。
2. 分割或拟合地面。
3. 选择或自动聚类目标物体区域。
4. 基于地面平面和目标区域生成 2.5D 高度图。
5. 将每个网格的高度乘以网格面积，累加得到体积。

体积单位为 `m3`，页面显示保留三位小数。

## 实时点云流程

实时点云适用于固定安装的 LiDAR，例如倒装在仓库顶部的 Unitree LiDAR 或固定位置的 RS-LiDAR。

### 1. 启动 ROS 和 WebSocket

Unitree 示例：

```bash
cd ~/point-cloud-web-viewer
source /opt/ros/humble/setup.bash
source unitree_lidar_ros2/install/setup.bash
source patchwork-plusplus-ros/install/setup.bash
source pointcloud_web_tools/install/setup.bash

ros2 launch pointcloud_web_tools unitree_webgl_viewer.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false \
  use_rviz:=true \
  accumulate_map:=true \
  map_window_seconds:=30 \
  map_voxel_size:=0.05
```

RS-LiDAR 示例：

```bash
cd ~/point-cloud-web-viewer
source /opt/ros/humble/setup.bash
source rslidar_sdk-v1.5.19/install/setup.bash
source patchwork-plusplus-ros/install/setup.bash
source pointcloud_web_tools/install/setup.bash

ros2 launch pointcloud_web_tools rslidar_webgl_viewer.launch.py \
  use_rviz:=true \
  accumulate_map:=true \
  map_window_seconds:=30 \
  map_voxel_size:=0.05
```

其中：

- `accumulate_map:=true` 表示后端发送滑动窗口点云，而不是单帧点云。
- `map_window_seconds:=30` 表示保留最近 30 秒点云。
- `map_voxel_size:=0.05` 表示滑动窗口点云的体素分辨率为 0.05 m。

实时体积计算建议使用滑动窗口点云，因为单帧 16 线或小型 LiDAR 点云较稀疏，不适合稳定分割和体积计算。

### 2. 启动网页

```bash
cd ~/point-cloud-web-viewer/webgl-pointcloud-viewer
python3 -m http.server 8082 --bind 0.0.0.0
```

本机打开：

```text
http://127.0.0.1:8082/
```

局域网设备打开：

```text
http://<ubuntu-ip>:8082/
```

如果同时运行 RS 和 Unitree，可使用：

```text
http://<ubuntu-ip>:8082/?sources=RS=8766,Unitree=8767
```

页面只有点击 `Live` 后才会接收实时点云。`Pause live` 会暂停接收，`Clear` 会清空当前点云和处理状态。

### 3. 实时点云处理步骤

1. 点击 `Live`，开始接收滑动窗口点云。
2. 如果 LiDAR 是倒装，点击 `Flip`，将点云翻转到真实世界方向。
3. 点击 `Fit ground`，后端对当前滑动窗口点云运行 Patchwork++。
4. 页面显示两类点：
   - 蓝色：地面点
   - 黄色：非地面点
5. 后端根据 Patchwork++ 分割出的地面点拟合地面平面。
6. 如需限制计算范围，点击 `ROI`，在 XY 平面上选择目标区域。
7. 点击 `Cluster`，选择：
   - `Stockpile`：煤堆等连续坡面物体
   - `Box object`：长方体货物等规则物体
8. 聚类结果会用红色显示。
9. 如有误选设备，可点击 `Exclude`：
   - 左键选择屏幕多边形点
   - 右键闭合
   - 按 `Enter` 确认
   - 被选中的点会从红色 cluster selection 和体积计算中排除
10. 点击 `Calculate`，计算目标区域体积。

实时模式下，体积计算基于当前滑动窗口中的点云，而不是只基于最新一帧。

## 静态 LAS 点云流程

静态 LAS 点云适用于手持 SLAM 扫描得到的仓库完整点云。

### 1. 加载 LAS

打开网页后点击 `Load LAS`，选择 `.las` 文件。加载成功后，页面会显示静态点云。

静态 LAS 不需要点击 `Live`。`Load LAS` 和 `Live` 是并列输入模式。

### 2. 结构过滤和地面处理

手持 SLAM 点云通常包含仓库顶棚、墙体、地面和目标物体。推荐流程：

1. 点击 `Filter structure`。
2. 前端对仓库结构进行过滤，去除明显的顶棚和墙体点。
3. 同时对剩余地面区域进行平面拟合，得到初始地面参数。
4. 如需更细的地面分割，点击 `Fit ground`，静态点云会上传给后端 Patchwork++ 处理。
5. Patchwork++ 返回地面点和非地面点，页面用不同颜色显示。

对于静态 LAS，`Fit ground` 会处理加载后的静态点云。为了避免点数过多导致处理过慢，静态 Patchwork++ 会先进行体素抽稀。

### 3. 目标区域选择和聚类

1. 点击 `ROI`，选择目标物体所在的大致区域。
2. 对煤堆，点击 `Cluster -> Stockpile`。
3. 对长方体货物，点击 `Cluster -> Box object`。
4. 聚类结果以红色显示。
5. 如有设备或非目标物体被误选，使用 `Exclude` 进行屏幕多边形排除。
6. 点击 `Calculate` 计算体积。

静态 LAS 模式下，体积计算同样基于地面平面和目标 cluster 的 2.5D 高度图。

## Cluster 逻辑

### Stockpile

`Stockpile` 用于煤堆这类不规则连续坡面物体。主要逻辑：

1. 根据地面平面计算每个点的离地高度。
2. 在 ROI 内生成 2.5D 高度图。
3. 选择高于地面且具有局部高度变化的候选网格。
4. 对候选网格进行连通区域聚类。
5. 过滤细长、悬空、突起设备等异常区域。
6. 对连续煤堆表面做小范围补全。
7. 输出最终煤堆 cluster。

Debug 中的主要阶段：

- `height map`：初始高度图候选区域。
- `footprint`：XY 平面煤堆轮廓区域。
- `protrusion outlier`：被识别为局部突起设备的区域。
- `final`：最终用于体积计算的目标区域。

### Box object

`Box object` 用于长方体货物等规则物体。主要逻辑更直接：

1. 根据地面平面计算离地高度。
2. 在 ROI 内选择高于地面的网格。
3. 做连通区域聚类。
4. 保留有效目标区域。

## 体积计算逻辑

体积计算使用 2.5D 高度图：

1. 对 cluster 内的点按 XY 网格分组。
2. 每个网格计算稳健表面高度。
3. 过滤过低高度，避免把地面噪声计入体积。
4. 对局部异常高度做限制，降低设备、噪点对体积的影响。
5. 体积累加公式：

```text
volume = sum(height_cell * grid_size * grid_size)
```

其中 `height_cell` 是该网格相对地面的高度，`grid_size` 使用当前体积网格大小，单位为米。

## 常用按钮说明

- `Live`：开始或暂停接收实时点云。
- `Load LAS`：加载静态 LAS 点云。
- `Flip`：翻转点云方向，适用于倒装 LiDAR。
- `Filter structure`：过滤仓库结构，主要用于静态 SLAM 点云。
- `Fit ground`：执行地面分割/拟合。
- `ROI`：选择目标区域。
- `Exclude`：屏幕多边形排除误选点。
- `Cluster`：选择目标物体类型并聚类。
- `Debug`：查看 Stockpile 聚类中间结果。
- `Calculate`：计算体积。
- `Undo`：撤销上一步操作。
- `Clear`：清空当前点云和处理状态。

## 注意事项

- 实时点云建议开启 `accumulate_map:=true`，并设置合适的 `map_window_seconds`。
- 倒装 LiDAR 需要先 `Flip`，再执行 `Fit ground`。
- 静态 LAS 点云较大时，`Fit ground` 可能较慢。
- `Exclude` 只影响 cluster selection 和体积计算，不会删除背景点云。
- 如果体积结果偏大，优先检查地面拟合、ROI、Cluster 和 Exclude。
- 如果体积结果偏小，优先检查 `Debug -> height map` 是否覆盖完整目标边缘。
