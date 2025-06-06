import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    package_name = 'e_dollyv2'
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)

    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    robot_description = {'robot_description': Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=true'
    ])}

    rsp = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description, {'use_sim_time': True}]
            )
        ]
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_params, {'use_sim_time': True}],
                remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
            )
        ]
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'e_dollyv2', '-z', '0.1'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    '--ros-args',
                    '-p',
                    f'config_file:={bridge_params}',
                ],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    ros_gz_image_bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                arguments=["/camera/image_raw"],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    teleop_keyboard = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')],
        parameters=[{'use_sim_time': True}]
    )

    joint_state_publisher = TimerAction(
        period=3.5,
        actions=[
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    ekf_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(get_package_share_directory('e_dollyv2'), 'config', 'ekf.yaml'),
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    launch_nodes = [
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        joint_state_publisher,
        ros_gz_bridge,
        ros_gz_image_bridge,
        teleop_keyboard,
        ekf_node
    ]

    if use_ros2_control == 'true':
        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                {'use_sim_time': True}
            ],
            output="screen"
        )
        delayed_controller_manager = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=3.0,
                        actions=[controller_manager]
                    )
                ],
            )
        )

        diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
        )
        joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
        )
        delayed_diff_drive_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[diff_drive_spawner],
            )
        )
        delayed_joint_broad_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[joint_broad_spawner],
            )
        )
        launch_nodes.extend([
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner
        ])

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Enable ros2_control and related nodes'
        ),
        OpaqueFunction(function=launch_setup)
    ])
