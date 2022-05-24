#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.


"""Launch Webots and the controllers."""

import os
import pathlib
import launch
import yaml
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'ur3e_environment'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    ur3e_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_ur5e_description.urdf')).read_text()
    ur3e_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    # Webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'factory.wbt'))

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_ROBOT_NAME` has to match the robot name in the world file.
    # You can check for more information at:
    # https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers
    ur3e_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR3e'},
        namespace='ur3e',
        parameters=[
            {'robot_description': ur3e_description},
            {'use_sim_time': True},
            ur3e_control_params
        ]
        # 'set_robot_state_publisher': True},
        # ]
    )



    # Control nodes
    ur3e_controller = Node(
        package=PACKAGE_NAME,
        executable='ur3e_driver',
        namespace='ur3e',
        output='screen'
    )


    return launch.LaunchDescription([
        webots,
        ur3e_controller,
        ur3e_driver,
        # ur3e_trajectory_controller_spawner,
        # ur3e_joint_state_broadcaster_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])