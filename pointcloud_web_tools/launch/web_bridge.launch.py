from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    mode = LaunchConfiguration("mode")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    voxel_size = LaunchConfiguration("voxel_size")
    point_stride = LaunchConfiguration("point_stride")
    include_intensity = LaunchConfiguration("include_intensity")
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument("output_topic", default_value="/rslidar_points_web"),
            DeclareLaunchArgument("mode", default_value="voxel"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.10"),
            DeclareLaunchArgument("point_stride", default_value="4"),
            DeclareLaunchArgument("include_intensity", default_value="false"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("address", default_value="0.0.0.0"),
            Node(
                package="pointcloud_web_tools",
                executable="pointcloud_downsampler",
                name="pointcloud_downsampler",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "mode": mode,
                        "publish_rate_hz": publish_rate_hz,
                        "voxel_size": voxel_size,
                        "point_stride": point_stride,
                        "include_intensity": include_intensity,
                    }
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "address": address,
                        "topic_whitelist": ["^/rslidar_points_web$", "^/tf$", "^/tf_static$"],
                        "send_buffer_limit": 1000000,
                    }
                ],
            ),
        ]
    )
