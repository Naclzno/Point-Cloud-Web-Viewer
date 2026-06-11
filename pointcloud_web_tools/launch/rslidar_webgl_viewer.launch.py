from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    voxel_size = LaunchConfiguration("voxel_size")
    accumulate_map = LaunchConfiguration("accumulate_map")
    map_voxel_size = LaunchConfiguration("map_voxel_size")
    map_window_seconds = LaunchConfiguration("map_window_seconds")
    ws_port = LaunchConfiguration("ws_port")
    ws_address = LaunchConfiguration("ws_address")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    bag_duration = LaunchConfiguration("bag_duration")
    use_rviz = LaunchConfiguration("use_rviz")

    rslidar_share = get_package_share_directory("rslidar_sdk")
    config_file = rslidar_share + "/config/config.yaml"
    rviz_config = rslidar_share + "/rviz/rviz2.rviz"

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("input_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.10"),
            DeclareLaunchArgument("accumulate_map", default_value="false"),
            DeclareLaunchArgument("map_voxel_size", default_value="0.10"),
            DeclareLaunchArgument("map_window_seconds", default_value="0.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("ws_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_output", default_value="/tmp/rslidar_recent_bag"),
            DeclareLaunchArgument("bag_duration", default_value="3.0"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                namespace="rslidar_sdk",
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                name="rslidar_sdk_node",
                output="screen",
                parameters=[{"config_path": config_file}],
            ),
            Node(
                package="pointcloud_web_tools",
                executable="pointcloud_ws_server",
                name="pointcloud_ws_server",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "voxel_size": ParameterValue(voxel_size, value_type=float),
                        "accumulate_map": ParameterValue(accumulate_map, value_type=bool),
                        "map_voxel_size": ParameterValue(map_voxel_size, value_type=float),
                        "map_window_seconds": ParameterValue(
                            map_window_seconds, value_type=float
                        ),
                        "port": ParameterValue(ws_port, value_type=int),
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
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
