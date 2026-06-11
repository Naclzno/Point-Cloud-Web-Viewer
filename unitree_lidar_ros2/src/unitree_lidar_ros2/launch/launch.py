import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    initialize_type_arg = DeclareLaunchArgument(
        'initialize_type',
        default_value='2',
        description='Lidar initialization type: 1 for serial, 2 for UDP.'
    )
    work_mode_arg = DeclareLaunchArgument(
        'work_mode',
        default_value='0',
        description='Lidar work mode. Use 0 for UDP mode and 8 for serial mode.'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial device path used when initialize_type is 1.'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='4000000',
        description='Serial baudrate used when initialize_type is 1.'
    )
    start_lidar_rotation_arg = DeclareLaunchArgument(
        'start_lidar_rotation',
        default_value='true',
        description='Call startLidarRotation after initialization.'
    )
    reset_lidar_after_set_mode_arg = DeclareLaunchArgument(
        'reset_lidar_after_set_mode',
        default_value='true',
        description='Call resetLidar after setting work mode.'
    )
    use_system_timestamp_arg = DeclareLaunchArgument(
        'use_system_timestamp',
        default_value='true',
        description='Use host system time for point cloud stamps. If false, use lidar hardware stamps.'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish Unitree vendor demo TFs.'
    )
    imu_quaternion_order_arg = DeclareLaunchArgument(
        'imu_quaternion_order',
        default_value='wxyz',
        description='Order of quaternion values provided by the Unitree SDK: wxyz or xyzw.'
    )
    imu_linear_acceleration_scale_arg = DeclareLaunchArgument(
        'imu_linear_acceleration_scale',
        default_value='1.0',
        description='Scale SDK IMU acceleration to ROS m/s^2.'
    )
    imu_angular_velocity_scale_arg = DeclareLaunchArgument(
        'imu_angular_velocity_scale',
        default_value='0.017453292519943295',
        description='Scale SDK IMU angular velocity to ROS rad/s. Default converts deg/s to rad/s.'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='6101',
        description='Lidar UDP port used when initialize_type is 2.'
    )
    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.1.62',
        description='Lidar UDP IP used when initialize_type is 2.'
    )
    local_port_arg = DeclareLaunchArgument(
        'local_port',
        default_value='6201',
        description='Local UDP port used when initialize_type is 2.'
    )
    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.1.2',
        description='Local UDP IP used when initialize_type is 2.'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz.'
    )
    save_cloud_txt_arg = DeclareLaunchArgument(
        'save_cloud_txt',
        default_value='false',
        description='Save parsed point cloud data to a txt file.'
    )
    cloud_txt_path_arg = DeclareLaunchArgument(
        'cloud_txt_path',
        default_value='/tmp/unitree_lidar_cloud.txt',
        description='Output txt file path used when cloud_txt_save_mode is overwrite_one_file.'
    )
    cloud_txt_save_mode_arg = DeclareLaunchArgument(
        'cloud_txt_save_mode',
        default_value='overwrite_one_file',
        description='Point cloud txt save mode: overwrite_one_file or separate_files.'
    )
    cloud_txt_dir_arg = DeclareLaunchArgument(
        'cloud_txt_dir',
        default_value='/tmp/unitree_lidar_cloud_frames',
        description='Output directory used when cloud_txt_save_mode is separate_files.'
    )
    cloud_txt_save_every_n_arg = DeclareLaunchArgument(
        'cloud_txt_save_every_n',
        default_value='1',
        description='Save one point cloud frame every N parsed frames.'
    )

    # Run unitree lidar
    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                
                {'initialize_type': ParameterValue(LaunchConfiguration('initialize_type'), value_type=int)},
                {'work_mode': ParameterValue(LaunchConfiguration('work_mode'), value_type=int)},
                {'use_system_timestamp': ParameterValue(LaunchConfiguration('use_system_timestamp'), value_type=bool)},
                {'start_lidar_rotation': ParameterValue(LaunchConfiguration('start_lidar_rotation'), value_type=bool)},
                {'reset_lidar_after_set_mode': ParameterValue(LaunchConfiguration('reset_lidar_after_set_mode'), value_type=bool)},
                {'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)},
                {'imu_quaternion_order': LaunchConfiguration('imu_quaternion_order')},
                {'imu_angular_velocity_scale': ParameterValue(LaunchConfiguration('imu_angular_velocity_scale'), value_type=float)},
                {'imu_linear_acceleration_scale': ParameterValue(LaunchConfiguration('imu_linear_acceleration_scale'), value_type=float)},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': LaunchConfiguration('serial_port')},
                {'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int)},

                {'lidar_port': ParameterValue(LaunchConfiguration('lidar_port'), value_type=int)},
                {'lidar_ip': LaunchConfiguration('lidar_ip')},
                {'local_port': ParameterValue(LaunchConfiguration('local_port'), value_type=int)},
                {'local_ip': LaunchConfiguration('local_ip')},
                
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},

                {'save_cloud_txt': ParameterValue(LaunchConfiguration('save_cloud_txt'), value_type=bool)},
                {'cloud_txt_save_mode': LaunchConfiguration('cloud_txt_save_mode')},
                {'cloud_txt_path': LaunchConfiguration('cloud_txt_path')},
                {'cloud_txt_dir': LaunchConfiguration('cloud_txt_dir')},
                {'cloud_txt_save_every_n': ParameterValue(LaunchConfiguration('cloud_txt_save_every_n'), value_type=int)},
                ]
    )

    # Run Rviz
    package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
    print("rviz_config_file = " + rviz_config_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    return LaunchDescription([
        initialize_type_arg,
        work_mode_arg,
        serial_port_arg,
        baudrate_arg,
        start_lidar_rotation_arg,
        reset_lidar_after_set_mode_arg,
        use_system_timestamp_arg,
        publish_tf_arg,
        imu_quaternion_order_arg,
        imu_angular_velocity_scale_arg,
        imu_linear_acceleration_scale_arg,
        lidar_port_arg,
        lidar_ip_arg,
        local_port_arg,
        local_ip_arg,
        use_rviz_arg,
        save_cloud_txt_arg,
        cloud_txt_save_mode_arg,
        cloud_txt_path_arg,
        cloud_txt_dir_arg,
        cloud_txt_save_every_n_arg,
        node1,
        rviz_node,
    ])
