#!/usr/bin/env python3
"""
Launch file for Emu Droid Gazebo Garden simulation

This launch file starts the complete simulation environment using the new Gazebo Garden including:
- Gazebo Garden world with testing environment
- Emu droid robot model with physics
- Camera sensors for stereo vision
- ROS 2 control interfaces for joint control
- Robot state publisher for TF tree

Usage:
    ros2 launch emu_droid_sim emu_gazebo_garden.launch.py
    ros2 launch emu_droid_sim emu_gazebo_garden.launch.py world:=empty.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    """Generate launch description for emu droid Gazebo Garden simulation."""

    # Get package directories
    pkg_sim_dir = '/home/bozman/mu-bot/sim'  # Direct path for now

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='emu_testing_world.sdf',
        description='Gazebo world file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='emu_droid',
        description='Name of the robot'
    )

    robot_x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial X position of robot'
    )

    robot_y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial Y position of robot'
    )

    robot_z_arg = DeclareLaunchArgument(
        'z',
        default_value='1.0',
        description='Initial Z position of robot'
    )

    # URDF file path
    urdf_file = os.path.join(pkg_sim_dir, 'urdf', 'emu_droid.urdf.xacro')

    # Process URDF through xacro
    robot_description_content = Command(['xacro ', urdf_file,
                                        ' robot_name:=', LaunchConfiguration('robot_name')])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Joint state publisher (for manual testing if needed)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition('false')  # Disabled by default since ros2_control handles it
    )

    # Start Gazebo Garden - using ExecuteProcess for better control
    gz_world_path = os.path.join(pkg_sim_dir, 'worlds', 'emu_testing_world.sdf')

    start_gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            gz_world_path,
            '-s'  # Start in headless mode by default due to WSL GUI issues
        ],
        output='screen',
        name='gazebo_garden'
    )

    # Alternative GUI launcher (enable manually if needed)
    start_gazebo_gui = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'export LIBGL_ALWAYS_SOFTWARE=1 && export QT_XCB_GL_INTEGRATION=none && gz sim {gz_world_path}'
        ],
        output='screen',
        name='gazebo_garden_gui',
        condition=IfCondition('false')  # Disabled by default due to WSL issues
    )    # Spawn robot in Gazebo Garden
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_emu_droid',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Bridge for robot state (joint states)
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_joint_states',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Load and start joint trajectory controller
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    load_leg_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_position_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    load_head_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_position_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # RViz visualization (optional)
    rviz_config_file = os.path.join(pkg_sim_dir, 'config', 'emu_droid_sim.rviz')

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition('false')  # Disabled by default, enable if needed
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        world_arg,
        gui_arg,
        headless_arg,
        robot_name_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,

        # Launch nodes
        robot_state_publisher,
        joint_state_publisher,
        start_gazebo,
        bridge_clock,
        bridge_joint_states,
        spawn_entity,

        # Controllers (delayed start to ensure robot is spawned first)
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_leg_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_leg_controller,
                on_exit=[load_head_controller],
            )
        ),

        # Optional RViz
        start_rviz,
    ])
