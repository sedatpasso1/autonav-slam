from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("autonav_slam")

    return LaunchDescription([
        DeclareLaunchArgument("lidar_topic", default_value="/velodyne_points"),
        DeclareLaunchArgument("imu_topic",   default_value="/imu/data"),
        DeclareLaunchArgument("voxel_size",  default_value="0.3"),
        DeclareLaunchArgument("use_rviz",    default_value="true"),

        Node(
            package="autonav_slam",
            executable="slam_node",
            name="autonav_slam",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg, "config", "params.yaml"]),
                {
                    "lidar_topic": LaunchConfiguration("lidar_topic"),
                    "imu_topic":   LaunchConfiguration("imu_topic"),
                    "voxel_size":  LaunchConfiguration("voxel_size"),
                },
            ],
        ),

        # RViz2 — harita ve odometry görselleştirme
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "config", "slam.rviz"])],
            condition=__import__("launch.conditions", fromlist=["IfCondition"])
                .IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
