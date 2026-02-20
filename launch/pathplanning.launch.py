# Copyright (c) 2024 Smart Rollerz e.V.
# All rights reserved.

import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description for the pathplanning node."""
    # Declare the 'debug' argument
    debug_arg = DeclareLaunchArgument(
        "debug", default_value="false", description="Enable debug mode"
    )
    crossing_interference_enabled_arg = DeclareLaunchArgument(
        "crossing_interference_enabled",
        default_value="false",
        description="Enable crossing interference",
    )

    # Define the path to the YAML configuration file
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("pathplanning"), "config", "ros_params.yaml"
        ),
        description="Path to the configuration file",
    )

    config_file = LaunchConfiguration("params_file")
    # Define the node
    pathplanning_node = Node(
        package="pathplanning",
        executable="path_planning_node",
        namespace="",
        name="path_planning_node",
        output="screen",
        parameters=[
            config_file,
            {
                "debug": LaunchConfiguration("debug"),
                "crossing_interference_enabled": LaunchConfiguration(
                    "crossing_interference_enabled"
                ),
            },
        ],
    )

    # Create and return the launch description
    return LaunchDescription(
        [
            params_file_arg,
            debug_arg,
            crossing_interference_enabled_arg,
            pathplanning_node,
        ]
    )
