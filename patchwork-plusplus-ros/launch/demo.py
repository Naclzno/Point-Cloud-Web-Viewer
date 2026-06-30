from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    output_cloud_topic = LaunchConfiguration("output_cloud_topic")
    output_ground_topic = LaunchConfiguration("output_ground_topic")
    output_nonground_topic = LaunchConfiguration("output_nonground_topic")
    params_file = PathJoinSubstitution([FindPackageShare("patchworkpp"), "config", "params.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("patchworkpp"), "rviz", "demo.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/kitti/velo/pointcloud"),
        DeclareLaunchArgument("output_cloud_topic", default_value="/ground_segmentation/cloud"),
        DeclareLaunchArgument("output_ground_topic", default_value="/ground_segmentation/ground"),
        DeclareLaunchArgument("output_nonground_topic", default_value="/ground_segmentation/nonground"),
        Node(
            package="patchworkpp",
            executable="demo",
            name="ground_segmentation",
            output="screen",
            parameters=[
                params_file,
                {
                    "cloud_topic": cloud_topic,
                    "output_cloud_topic": output_cloud_topic,
                    "output_ground_topic": output_ground_topic,
                    "output_nonground_topic": output_nonground_topic,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_patchworkpp",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
