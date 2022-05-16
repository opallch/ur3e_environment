from setuptools import setup

package_name = 'ur3e_environment'

"""Cocobots simulation package setup file. Based on webots_ros2_universal_robot"""

from setuptools import setup

package_name = 'ur3e_environment'
data_files = [
    ('share/' + package_name + '/controllers/ur_controller', [
        'controllers/ur_controller/coco.py',
        'controllers/ur_controller/composer.py',
        'controllers/ur_controller/ur_controller.py',
        'controllers/ur_controller/spec.yml',
        'controllers/ur_controller/writers.py'
    ]),
    ('share/' + package_name + '/launch', [
        'launch/cocobots_launch.py',
    ]),
    # ('share/' + package_name + '/libraries'),
    # ('share/' + package_name + '/plugins', [
    #     'plugins/physics',
    #     'plugins/remote_controls',
    #     'plugins/robot_windows'
    # ]),
    ('share/' + package_name + '/protos', [
        'protos/Board.proto',
        'protos/cross_recess_screw_M10_L20.proto',
        'protos/Hex_Nut_M15.proto',
        'protos/Hex_Screw_M16.proto',
        'protos/Plate.proto',
        'protos/Rect_Nut_M16.proto',
        'protos/Rect_Washer_M15.proto',
        'protos/Round_Washer_M15.proto',
        'protos/SocketHead_Screw_M16.proto',
        'protos/Triang_Screw_M28.proto',
        'protos/UR3e_VGC10.proto'       
    ]),
    ('share/' + package_name + '/resource', [
        'resource/ur3e_environment',
        'resource/webots_ur5e_description.urdf',
        'resource/ros2_control_config.yaml'
    ]),
    ('share/' + package_name + '/worlds', [
        'worlds/factory.wbt'
        # 'worlds/.factory.wbproj',
    ]),
    
    ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
]

setup(
    name=package_name,
    version='1.2.2',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexnic31@gmail.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Universal Robots'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Universal Robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
            'cocobots_driver = ur3e_environment.cocobots_driver:main',
        ]
    }
)