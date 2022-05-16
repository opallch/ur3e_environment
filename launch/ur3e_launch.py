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
    ur5e_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_ur5e_description.urdf')).read_text()
    ur5e_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    # Webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'factory.wbt'))

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_ROBOT_NAME` has to match the robot name in the world file.
    # You can check for more information at:
    # https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers
    ur5e_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5e'},
        namespace='ur5e',
        parameters=[
            {'robot_description': ur5e_description},
            {'use_sim_time': True},
            ur5e_control_params
        ]
        # 'set_robot_state_publisher': True},
        # ]
    )

    controller_manager_timeout = ['--controller-manager-timeout', '75']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'
    ur5e_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    ur5e_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )

    # Control nodes
    cocobots_driver = Node(
        package=PACKAGE_NAME,
        executable='cocobots_driver',
        namespace='ur5e',
        output='screen'
    )


    # # The Ros2Supervisor node is a special node interacting with the simulation.
    # # For example, it publishes the /clock topic of the simulation or permits to spawn robot from URDF files.
    # # By default, its respawn option is set at True.
    # ros2_supervisor = Ros2SupervisorLauncher() # Only with the develop branch!


    # # Often we want to publish robot transforms, so we use the `robot_state_publisher` node for that.
    # # If robot model is not specified in the URDF file then Webots can help us with the URDF exportation feature.
    # # Since the exportation feature is available only once the simulation has started and the `robot_state_publisher` node requires a `robot_description` parameter before we have to specify a dummy robot.
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': '<robot name=""><link name=""/></robot>'
    #     }],
    # )

    return launch.LaunchDescription([
        webots,
        cocobots_driver,
        ur5e_driver,
        ur5e_trajectory_controller_spawner,
        ur5e_joint_state_broadcaster_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])