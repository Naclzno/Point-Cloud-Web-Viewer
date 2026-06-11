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
  map_voxel_size:=0.05
```

`accumulate_map:=true` sends the accumulated voxel map instead of only the latest
frame. `map_voxel_size` controls map resolution. `map_window_seconds:=0.0` keeps
the map until the process exits; set it to a number of seconds to keep a sliding
time window of recently observed voxels.

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
