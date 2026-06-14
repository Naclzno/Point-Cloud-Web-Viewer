from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    initialize_type = LaunchConfiguration("initialize_type")
    work_mode = LaunchConfiguration("work_mode")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    start_lidar_rotation = LaunchConfiguration("start_lidar_rotation")
    reset_lidar_after_set_mode = LaunchConfiguration("reset_lidar_after_set_mode")
    use_system_timestamp = LaunchConfiguration("use_system_timestamp")
    publish_tf = LaunchConfiguration("publish_tf")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cloud_frame = LaunchConfiguration("cloud_frame")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_frame = LaunchConfiguration("imu_frame")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    input_qos = LaunchConfiguration("input_qos")
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
    ws_port = LaunchConfiguration("ws_port")
    ws_address = LaunchConfiguration("ws_address")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    bag_duration = LaunchConfiguration("bag_duration")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = get_package_share_directory("pointcloud_web_tools") + "/rviz/unitree_view.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument("initialize_type", default_value="1"),
            DeclareLaunchArgument("work_mode", default_value="8"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="4000000"),
            DeclareLaunchArgument("start_lidar_rotation", default_value="true"),
            DeclareLaunchArgument("reset_lidar_after_set_mode", default_value="false"),
            DeclareLaunchArgument("use_system_timestamp", default_value="true"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("cloud_topic", default_value="unilidar/cloud"),
            DeclareLaunchArgument("cloud_frame", default_value="unilidar_lidar"),
            DeclareLaunchArgument("imu_topic", default_value="unilidar/imu"),
            DeclareLaunchArgument("imu_frame", default_value="unilidar_imu"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("input_qos", default_value="sensor_data"),
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
            DeclareLaunchArgument("ws_port", default_value="8767"),
            DeclareLaunchArgument("ws_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_output", default_value="/tmp/unitree_recent_bag"),
            DeclareLaunchArgument("bag_duration", default_value="3.0"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                package="unitree_lidar_ros2",
                executable="unitree_lidar_ros2_node",
                name="unitree_lidar_ros2_node",
                output="screen",
                parameters=[
                    {
                        "initialize_type": ParameterValue(initialize_type, value_type=int),
                        "work_mode": ParameterValue(work_mode, value_type=int),
                        "serial_port": serial_port,
                        "baudrate": ParameterValue(baudrate, value_type=int),
                        "start_lidar_rotation": ParameterValue(
                            start_lidar_rotation, value_type=bool
                        ),
                        "reset_lidar_after_set_mode": ParameterValue(
                            reset_lidar_after_set_mode, value_type=bool
                        ),
                        "use_system_timestamp": ParameterValue(
                            use_system_timestamp, value_type=bool
                        ),
                        "publish_tf": ParameterValue(publish_tf, value_type=bool),
                        "range_min": 0.0,
                        "range_max": 100.0,
                        "cloud_scan_num": 18,
                        "cloud_topic": cloud_topic,
                        "cloud_frame": cloud_frame,
                        "imu_topic": imu_topic,
                        "imu_frame": imu_frame,
                        "imu_quaternion_order": "wxyz",
                        "imu_angular_velocity_scale": 0.017453292519943295,
                        "imu_linear_acceleration_scale": 1.0,
                        "save_cloud_txt": False,
                    }
                ],
            ),
            Node(
                package="pointcloud_web_tools",
                executable="pointcloud_ws_server",
                name="unitree_pointcloud_ws_server",
                output="screen",
                parameters=[
                    {
                        "input_topic": cloud_topic,
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
                    cloud_topic,
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
