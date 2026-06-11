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
