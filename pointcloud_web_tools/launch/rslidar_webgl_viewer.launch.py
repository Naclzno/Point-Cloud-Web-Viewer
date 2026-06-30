from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    input_qos = LaunchConfiguration("input_qos")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    voxel_size = LaunchConfiguration("voxel_size")
    accumulate_map = LaunchConfiguration("accumulate_map")
    map_voxel_size = LaunchConfiguration("map_voxel_size")
    map_window_seconds = LaunchConfiguration("map_window_seconds")
    enable_volume = LaunchConfiguration("enable_volume")
    volume_update_hz = LaunchConfiguration("volume_update_hz")
    volume_min_height = LaunchConfiguration("volume_min_height")
    volume_roi_min_x = LaunchConfiguration("volume_roi_min_x")
    volume_roi_max_x = LaunchConfiguration("volume_roi_max_x")
    volume_roi_min_y = LaunchConfiguration("volume_roi_min_y")
    volume_roi_max_y = LaunchConfiguration("volume_roi_max_y")
    ground_plane_a = LaunchConfiguration("ground_plane_a")
    ground_plane_b = LaunchConfiguration("ground_plane_b")
    ground_plane_c = LaunchConfiguration("ground_plane_c")
    ground_plane_d = LaunchConfiguration("ground_plane_d")
    patchwork_update_hz = LaunchConfiguration("patchwork_update_hz")
    ws_port = LaunchConfiguration("ws_port")
    ws_address = LaunchConfiguration("ws_address")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    bag_duration = LaunchConfiguration("bag_duration")
    use_rviz = LaunchConfiguration("use_rviz")

    rslidar_share = get_package_share_directory("rslidar_sdk")
    patchwork_params_file = get_package_share_directory("patchworkpp") + "/config/params.yaml"
    config_file = rslidar_share + "/config/config.yaml"
    rviz_config = rslidar_share + "/rviz/rviz2.rviz"

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("input_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument("input_qos", default_value="default"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.10"),
            DeclareLaunchArgument("accumulate_map", default_value="true"),
            DeclareLaunchArgument("map_voxel_size", default_value="0.10"),
            DeclareLaunchArgument("map_window_seconds", default_value="15.0"),
            DeclareLaunchArgument("enable_volume", default_value="false"),
            DeclareLaunchArgument("volume_update_hz", default_value="1.0"),
            DeclareLaunchArgument("volume_min_height", default_value="0.05"),
            DeclareLaunchArgument("volume_roi_min_x", default_value="-1000.0"),
            DeclareLaunchArgument("volume_roi_max_x", default_value="1000.0"),
            DeclareLaunchArgument("volume_roi_min_y", default_value="-1000.0"),
            DeclareLaunchArgument("volume_roi_max_y", default_value="1000.0"),
            DeclareLaunchArgument("ground_plane_a", default_value="0.0"),
            DeclareLaunchArgument("ground_plane_b", default_value="0.0"),
            DeclareLaunchArgument("ground_plane_c", default_value="1.0"),
            DeclareLaunchArgument("ground_plane_d", default_value="0.0"),
            DeclareLaunchArgument("patchwork_update_hz", default_value="2.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("ws_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_output", default_value="/tmp/rslidar_recent_bag"),
            DeclareLaunchArgument("bag_duration", default_value="3.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
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
                name="rslidar_pointcloud_ws_server",
                output="screen",
                parameters=[
                    patchwork_params_file,
                    {
                        "input_topic": input_topic,
                        "input_qos": input_qos,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "voxel_size": ParameterValue(voxel_size, value_type=float),
                        "accumulate_map": ParameterValue(accumulate_map, value_type=bool),
                        "map_voxel_size": ParameterValue(map_voxel_size, value_type=float),
                        "map_window_seconds": ParameterValue(
                            map_window_seconds, value_type=float
                        ),
                        "enable_volume": ParameterValue(enable_volume, value_type=bool),
                        "volume_update_hz": ParameterValue(volume_update_hz, value_type=float),
                        "volume_min_height": ParameterValue(volume_min_height, value_type=float),
                        "volume_roi_min_x": ParameterValue(volume_roi_min_x, value_type=float),
                        "volume_roi_max_x": ParameterValue(volume_roi_max_x, value_type=float),
                        "volume_roi_min_y": ParameterValue(volume_roi_min_y, value_type=float),
                        "volume_roi_max_y": ParameterValue(volume_roi_max_y, value_type=float),
                        "ground_plane_a": ParameterValue(ground_plane_a, value_type=float),
                        "ground_plane_b": ParameterValue(ground_plane_b, value_type=float),
                        "ground_plane_c": ParameterValue(ground_plane_c, value_type=float),
                        "ground_plane_d": ParameterValue(ground_plane_d, value_type=float),
                        "patchwork_update_hz": ParameterValue(
                            patchwork_update_hz, value_type=float
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
