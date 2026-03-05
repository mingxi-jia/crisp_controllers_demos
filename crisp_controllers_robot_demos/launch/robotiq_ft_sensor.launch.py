"""Standalone launch file for Robotiq FT sensor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robotiq_ft_sensor = Node(
        package="robotiq_ft_sensor_hardware",
        executable="robotiq_ft_sensor_standalone_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {"max_retries": LaunchConfiguration("max_retries")},
            {"read_rate": LaunchConfiguration("read_rate")},
            {"ftdi_id": LaunchConfiguration("ftdi_id")},
            {"frame_id": LaunchConfiguration("frame_id")},
        ],
        output="screen",
    )

    return [robotiq_ft_sensor]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="ft"),
            DeclareLaunchArgument("max_retries", default_value="100"),
            DeclareLaunchArgument("read_rate", default_value="100"),
            DeclareLaunchArgument("ftdi_id", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="robotiq_ft_frame_id"),
            OpaqueFunction(function=launch_setup),
        ]
    )
