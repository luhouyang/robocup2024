import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'main_package'
    world_name = 'world_sim.world'
    rviz_name = 'rviz_sim.rviz'

    declare_world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'worlds',
                                   world_name),
        description='Full path to .world file')

    declare_rviz_launch_arg = DeclareLaunchArgument(
        name='rviz',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config',
                                   rviz_name),
        description='Full path to .rviz file')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name),
                         'launch',
                         'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            #    'use_ros2_control': 'true'
        }.items())

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    [
                        os.path.join(get_package_share_directory(package_name),
                                     'config',
                                     rviz_name)
                    ]
                ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch',
                         'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(get_package_share_directory(package_name),
                                  'worlds',
                                  world_name),
            'extra_gazebo_args': '--ros-args --params-file ' +
            os.path.join(get_package_share_directory(package_name),
                         'params',
                         'gazebo_params.yaml')
        }.items())

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic',
                   'robot_description',
                   '-entity',
                   'main_package'],
        output='screen')

    # diff_drive_spawner = Node(package="controller_manager",
    #                           executable="spawner",
    #                           arguments=["diff_cont"])

    # joint_broad_spawner = Node(package="controller_manager",
    #                            executable="spawner",
    #                            arguments=["joint_broad"])

    return LaunchDescription([
        declare_world_launch_arg,
        declare_rviz_launch_arg,
        rsp,
        rviz,
        gazebo,
        spawn_entity
        # , diff_drive_spawner, joint_broad_spawner
    ])
