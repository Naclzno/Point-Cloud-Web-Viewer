from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare("patchworkpp"), "config", "params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("sequence", default_value="00"),
        DeclareLaunchArgument("init_idx", default_value="0"),
        DeclareLaunchArgument("data_path", default_value="/../../seungjae_ssd/data/SemanticKITTI/sequences"),
        DeclareLaunchArgument("stop_per_each_frame", default_value="false"),
        Node(
            package="patchworkpp",
            executable="video",
            name="video",
            output="screen",
            parameters=[
                params_file,
                {
                    "sequence": LaunchConfiguration("sequence"),
                    "init_idx": LaunchConfiguration("init_idx"),
                    "data_path": LaunchConfiguration("data_path"),
                    "stop_per_each_frame": LaunchConfiguration("stop_per_each_frame"),
                },
            ],
        ),
    ])
