from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("autonav_slam")

    return LaunchDescription([
        DeclareLaunchArgument("lidar_topic",  default_value="/lidar/points"),
        DeclareLaunchArgument("imu_topic",    default_value="/imu/data"),
        DeclareLaunchArgument("voxel_size",   default_value="0.1"),
        DeclareLaunchArgument("lidar_type",   default_value="velodyne"),

        Node(
            package="autonav_slam",
            executable="fast_lio2_node",
            name="fast_lio2_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg, "config", "fast_lio2_params.yaml"]),
                {
                    "lidar_topic": LaunchConfiguration("lidar_topic"),
                    "imu_topic":   LaunchConfiguration("imu_topic"),
                    "voxel_size":  LaunchConfiguration("voxel_size"),
                    "lidar_type":  LaunchConfiguration("lidar_type"),
                },
            ],
        ),
    ])
