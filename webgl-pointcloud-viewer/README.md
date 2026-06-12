# WebGL Point Cloud Viewer

Minimal browser viewer for the `pointcloud_ws_server` binary stream.

The ROS node subscribes to `/rslidar_points`, voxel-filters each frame, and sends packed
`float32 x/y/z` points over WebSocket.

## Run

Start the ROS side:

```bash
cd ~/point-cloud-web-viewer
source /opt/ros/humble/setup.bash
source pointcloud_web_tools/install/setup.bash
ros2 launch pointcloud_web_tools webgl_viewer.launch.py
```

Tune the stream:

```bash
ros2 launch pointcloud_web_tools webgl_viewer.launch.py voxel_size:=0.10 publish_rate_hz:=5.0
```

Enable accumulated mapping when the LiDAR is fixed:

```bash
ros2 launch pointcloud_web_tools rslidar_webgl_viewer.launch.py \
  accumulate_map:=true \
  map_window_seconds:=15 \
  map_voxel_size:=0.10
```

`accumulate_map:=true` sends the accumulated voxel map instead of only the latest
frame. `map_voxel_size` controls map resolution. `map_window_seconds:=0.0` keeps
the map until the process exits; set it to a number of seconds to keep a sliding
time window of recently observed voxels.

## Web volume workflow

For an interactive stockpile measurement, launch the WebSocket server with
`accumulate_map:=true` and a finite `map_window_seconds`. The browser receives
that sliding-window map, so the volume is computed from recent accumulated points
instead of a single LiDAR frame.

In the page:

1. Click `Fit ground` after the warehouse floor is visible. The fitted plane is
   sent to `pointcloud_ws_server` and shown as `ground_plane_a/b/c/d` launch
   parameters.
2. Click `ROI`, then click two points in the cloud to define the opposite
   corners of the stockpile area.
3. Click `Calculate` to send the ROI to `pointcloud_ws_server` and start
   server-side volume output.
4. Click `Undo` to remove the ROI, restore the full point cloud, and disable
   server-side volume output.

Enable stockpile volume display:

```bash
ros2 launch pointcloud_web_tools rslidar_webgl_viewer.launch.py \
  accumulate_map:=true \
  map_window_seconds:=15 \
  map_voxel_size:=0.10 \
  enable_volume:=true \
  volume_roi_min_x:=-10 \
  volume_roi_max_x:=10 \
  volume_roi_min_y:=-10 \
  volume_roi_max_y:=10
```

The volume estimator uses the accumulated map, a fixed ground plane, and a 2D
height grid. `map_voxel_size` controls both accumulated-map resolution and volume
grid size. The default ground plane is `z=0`, encoded as
`ground_plane_a:=0 ground_plane_b:=0 ground_plane_c:=1 ground_plane_d:=0`.
Set these plane coefficients after calibrating the warehouse floor.

Start the RS-LiDAR pipeline:

```bash
cd ~/point-cloud-web-viewer
source /opt/ros/humble/setup.bash
source rslidar_sdk-v1.5.19/install/setup.bash
source pointcloud_web_tools/install/setup.bash
ros2 launch pointcloud_web_tools rslidar_webgl_viewer.launch.py
```

The RS launch starts `rslidar_sdk_node`, publishes `/rslidar_points`, streams it to
the WebGL viewer. Set `use_rviz:=true` to also open RViz. Set `record_bag:=true`
to keep the latest 3-second bag at `/tmp/rslidar_recent_bag`.

Start the Unitree serial pipeline:

```bash
cd ~/point-cloud-web-viewer
source /opt/ros/humble/setup.bash
source unitree_lidar_ros2/install/setup.bash
source pointcloud_web_tools/install/setup.bash
ros2 launch pointcloud_web_tools unitree_webgl_viewer.launch.py \
  initialize_type:=1 \
  work_mode:=8 \
  serial_port:=/dev/ttyACM0 \
  baudrate:=4000000 \
  start_lidar_rotation:=true \
  reset_lidar_after_set_mode:=false
```

The Unitree launch subscribes to `unilidar/cloud` by default. Set `use_rviz:=true`
to also open RViz. Set `record_bag:=true` to keep the latest 3-second bag at
`/tmp/unitree_recent_bag`.

Serve the web page:

```bash
cd ~/point-cloud-web-viewer/webgl-pointcloud-viewer
python3 -m http.server 8081 --bind 0.0.0.0
```

Open:

```text
http://127.0.0.1:8081/
```

From another device on the same LAN:

```text
http://192.168.1.162:8081/
```

The page connects to `ws://<page-host>:8766` by default. Override it with:

```text
http://127.0.0.1:8081/?ws=ws://127.0.0.1:8766
```
