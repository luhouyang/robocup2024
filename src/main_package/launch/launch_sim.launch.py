import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node


def generate_launch_description():

    #  variables
    package_name = 'main_package'
    world_name = 'world_sim.world'
    rviz_name = 'rviz_sim.rviz'
    params_file_name = 'mapper_params_online_async.yaml'

    # launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    slam_params_file = LaunchConfiguration('params_file')

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

    robot_description_config = Command(
        ['xacro ',
         xacro_file,
         ' sim_mode:=',
         use_sim_time])

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time,
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

    gz_params_file = os.path.join(get_package_share_directory(package_name),
                                  'params',
                                  'gazebo_params.yaml')

    gz_world_file = os.path.join(get_package_share_directory(package_name),
                                 'worlds',
                                 world_name)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch',
                         'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gz_params_file,
            'world': gz_world_file
        }.items())

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic',
                   'robot_description',
                   '-entity',
                   'main_package'],
        output='screen')

    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])

    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'robot_config.yaml')

    controller_manager = Node(package='controller_manager',
                              executable='ros2_control_node',
                              name='controller_manager',
                              output='screen',
                              parameters=[{
                                  'robot_description': robot_description
                              },
                                          controller_params_file])

    load_omni_wheel_controller = Node(package="controller_manager",
                                      executable="spawner",
                                      arguments=["omni_wheel_controller"])

    load_joint_state_controller = Node(package="controller_manager",
                                       executable="spawner",
                                       arguments=["joint_state_broadcaster"])

    delayed_controller_manager = TimerAction(period=1.0,
                                             actions=[controller_manager])

    delayed_load_omni_wheel_controller_handler = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[load_omni_wheel_controller]))

    delayed_load_joint_state_controller_handler = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[load_joint_state_controller]))

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        remappings=[
            ('/cmd_vel_stamped',
             '/omni_wheel_controller/cmd_vel'),
        ],
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name),
                         'launch',
                         'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items())

    twist_mux_params = os.path.join(get_package_share_directory(package_name),
                                    'config',
                                    'twist_mux.yaml')
    twist_mux = Node(package="twist_mux",
                     executable="twist_mux",
                     parameters=[twist_mux_params,
                                 {
                                     'use_sim_time': True
                                 }],
                     remappings=[('/cmd_vel_out',
                                  '/omni_wheel_controller/cmd_vel')])

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("slam_toolbox")),
            '/launch',
            '/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items())

    realsense = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("realsense2_camera")),
        '/launch',
        '/rs_launch.py'
    ]),
                                         launch_arguments={
                                             'pointcloud.enable': 'true',
                                         }.items())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control',
                              default_value='true',
                              description='Use ros2_control if true'),
        DeclareLaunchArgument('publish_rate',
                              default_value='1000.0',
                              description='Publish rate'), # Bridge
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(
                                  get_package_share_directory("main_package"),
                                  'config',
                                  params_file_name),
                              description='Full path to params file'),
        declare_world_launch_arg,
        declare_rviz_launch_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delayed_controller_manager,
        delayed_load_omni_wheel_controller_handler,
        delayed_load_joint_state_controller_handler,
        velocity_converter,
        rviz,
        joystick,
        slam_toolbox,
        realsense,
    ])
