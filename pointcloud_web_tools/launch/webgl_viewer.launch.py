from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    voxel_size = LaunchConfiguration("voxel_size")
    ws_port = LaunchConfiguration("ws_port")
    ws_address = LaunchConfiguration("ws_address")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    bag_duration = LaunchConfiguration("bag_duration")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.10"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("ws_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("bag_output", default_value="/tmp/rslidar_recent_bag"),
            DeclareLaunchArgument("bag_duration", default_value="3.0"),
            Node(
                package="pointcloud_web_tools",
                executable="pointcloud_ws_server",
                name="pointcloud_ws_server",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "publish_rate_hz": publish_rate_hz,
                        "voxel_size": voxel_size,
                        "port": ws_port,
                        "address": ws_address,
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "pointcloud_web_tools",
                    "recent_bag_recorder",
                    "--topic",
                    input_topic,
                    "--output",
                    bag_output,
                    "--duration",
                    bag_duration,
                ],
                output="screen",
                condition=IfCondition(record_bag),
            ),
        ]
    )
