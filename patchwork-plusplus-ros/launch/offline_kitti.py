from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare("patchworkpp"), "config", "params.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("patchworkpp"), "rviz", "patchworkpp_viz.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument("sequence", default_value="00"),
        DeclareLaunchArgument("init_idx", default_value="0"),
        DeclareLaunchArgument("data_path", default_value="/../../seungjae_ssd/data/SemanticKITTI/sequences"),
        DeclareLaunchArgument("output_csvpath", default_value="/data/patchworkpp/"),
        DeclareLaunchArgument("save_csv_file", default_value="true"),
        DeclareLaunchArgument("stop_per_each_frame", default_value="true"),
        Node(
            package="patchworkpp",
            executable="offline_kitti",
            name="offline_kitti",
            output="screen",
            parameters=[
                params_file,
                {
                    "algorithm": "patchworkpp",
                    "sequence": LaunchConfiguration("sequence"),
                    "init_idx": LaunchConfiguration("init_idx"),
                    "data_path": LaunchConfiguration("data_path"),
                    "output_csvpath": LaunchConfiguration("output_csvpath"),
                    "save_csv_file": LaunchConfiguration("save_csv_file"),
                    "stop_per_each_frame": LaunchConfiguration("stop_per_each_frame"),
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_patchworkpp_offline",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
