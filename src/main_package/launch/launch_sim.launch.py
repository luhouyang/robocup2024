import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'main_package'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch',
                         'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            #    'use_ros2_control': 'true'
        }.items())

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch',
                         'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args':
            '--ros-args --params-file ' +
            os.path.join(get_package_share_directory(package_name), 'params',
                         'gazebo_params.yaml')
        }.items())

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'main_package'],
        output='screen')

    # diff_drive_spawner = Node(package="controller_manager",
    #                           executable="spawner",
    #                           arguments=["diff_cont"])

    # joint_broad_spawner = Node(package="controller_manager",
    #                            executable="spawner",
    #                            arguments=["joint_broad"])

    return LaunchDescription([
        rsp, gazebo, spawn_entity
        # , diff_drive_spawner, joint_broad_spawner
    ])
