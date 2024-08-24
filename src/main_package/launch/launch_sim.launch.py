import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

import launch_ros.descriptions
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    #  variables
    package_name = 'main_package'
    world_name = 'world_sim.world'
    rviz_name = 'rviz_sim.rviz'

    # launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

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
    # robot_description_config = Command([
    #     'xacro ',
    #     xacro_file,
    #     ' use_ros2_control:=',
    #     use_ros2_control,
    #     ' sim_mode:=',
    #     use_sim_time
    # ])

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time,
        'use_ros2_control': use_ros2_control
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch',
                         'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' +
            os.path.join(get_package_share_directory(package_name),
                         'params',
                         'gazebo_params.yaml'),
            'world': os.path.join(get_package_share_directory(package_name),
                                  'worlds',
                                  world_name),
        }.items())

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic',
                   'robot_description',
                   '-entity',
                   'main_package'],
        output='screen')

    # robot_description = launch_ros.descriptions.ParameterValue(Command([
    #     'ros2 param get --hide-type /robot_state_publisher robot_description'
    # ]),
    #                                                            value_type=str)

    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])

    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'robot_config.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        },
                    controller_params_file]
        # parameters=[{
        #     'controller_params_file': controller_params_file
        # }],
        # remappings=[
        #     ('/controller_manager/robot_description',
        #      '/robot_description'),
        #     # ('/controller_manager/load_controller',
        #     #  '/load_controller')
        # ]
    )

    load_omni_wheel_controller = Node(package="controller_manager",
                                      executable="spawner",
                                      arguments=["omni_wheel_controller"])

    load_joint_state_controller = Node(package="controller_manager",
                                       executable="spawner",
                                       arguments=["joint_state_broadcaster"])

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('ros_gz_sim'),
    #                      'launch'),
    #         '/gz_sim.launch.py'
    #     ]),
    #     launch_arguments=[
    #         ('world',
    #          os.path.join(get_package_share_directory(package_name),
    #                       'worlds',
    #                       world_name)),
    #         ('gz_args',
    #          [
    #              ' -r -v 4 empty.sdf',
    #          ]),
    #         # ('publish_rate',
    #         #  LaunchConfiguration('publish_rate'))
    #     ])

    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=['-topic',
    #                'robot_description',
    #                '-entity',
    #                'main_package'],
    # )

    # load_joint_state_controller = ExecuteProcess(cmd=[
    #     'ros2',
    #     'control',
    #     'load_controller',
    #     '--set-state',
    #     'active',
    #     'joint_state_broadcaster'
    # ],
    #                                              output='screen')

    # load_omni_wheel_controller = ExecuteProcess(cmd=[
    #     'ros2',
    #     'control',
    #     'load_controller',
    #     '--set-state',
    #     'active',
    #     'omni_wheel_controller'
    # ],
    #                                             output='screen')

    delayed_controller_manager = TimerAction(period=1.0,
                                             actions=[controller_manager])

    delayed_load_omni_wheel_controller_handler = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[load_omni_wheel_controller]))
    # 'world': os.path.join(get_package_share_directory(package_name),
    #                       'worlds',
    #                       world_name),
    delayed_load_joint_state_controller_handler = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[load_joint_state_controller]))

    # Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/camera_1/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #         '/camera_1/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    #         # '/camera_2/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #         # '/camera_2/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    #     ],
    #     output='screen')

    # bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=[
    #         '/camera_1/image_raw',
    #         '/camera_2/image_raw',
    #     ],
    #     output='screen',
    # )

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
        DeclareLaunchArgument('use_ros2_control',
                              default_value='true',
                              description='Use ros2_control if true'),
        DeclareLaunchArgument('publish_rate',
                              default_value='1000.0',
                              description='Publish rate'),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(target_action=controller_manager,
        #                                 on_exit=load_joint_state_controller)),
        # RegisterEventHandler(event_handler=OnProcessExit(
        #     target_action=controller_manager,
        #     on_exit=[load_omni_wheel_controller],
        # )),
        declare_world_launch_arg,
        declare_rviz_launch_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delayed_controller_manager,
        delayed_load_omni_wheel_controller_handler,
        delayed_load_joint_state_controller_handler,
        # bridge,
        velocity_converter,
        rviz,
    ])
