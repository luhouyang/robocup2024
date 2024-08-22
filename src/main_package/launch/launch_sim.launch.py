import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    #  variables
    package_name = 'main_package'
    world_name = 'world_sim.world'
    rviz_name = 'rviz_sim.rviz'

    # launch args
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,
                              'description',
                              'robot',
                              'robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file,
                                                  mappings={'use_sim': 'true'})

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config.toprettyxml(indent='  ')
        # 'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      output='screen',
                                      parameters=[params])

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

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('gazebo_ros'),
    #                      'launch',
    #                      'gazebo.launch.py')
    #     ]),
    #     launch_arguments={
    #         'extra_gazebo_args': '--ros-args --params-file ' +
    #         os.path.join(get_package_share_directory(package_name),
    #                      'params',
    #                      'gazebo_params.yaml'),
    #         'world': os.path.join(get_package_share_directory(package_name),
    #                               'worlds',
    #                               world_name),
    #     }.items())

    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-topic',
    #                'robot_description',
    #                '-entity',
    #                'main_package'],
    #     output='screen')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('world',
             os.path.join(get_package_share_directory(package_name),
                          'worlds',
                          world_name)),
            ('gz_args',
             [' -r -v 4 empty.sdf'])
        ])

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic',
                   'robot_description',
                   '-entity',
                   'main_package'],
    )

    load_joint_state_controller = ExecuteProcess(cmd=[
        'ros2',
        'control',
        'load_controller',
        '--set-state',
        'active',
        'joint_state_broadcaster'
    ],
                                                 output='screen')

    load_omni_wheel_controller = ExecuteProcess(cmd=[
        'ros2',
        'control',
        'load_controller',
        '--set-state',
        'active',
        'omni_wheel_controller'
    ],
                                                output='screen')

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        remappings=[
            ('/cmd_vel_stamped',
             '/omni_wheel_controller/cmd_vel'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use sim time if true'),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_entity,
                                        on_exit=load_joint_state_controller)),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_omni_wheel_controller],
        )),
        declare_world_launch_arg,
        declare_rviz_launch_arg,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        velocity_converter,
        rviz,
    ])
