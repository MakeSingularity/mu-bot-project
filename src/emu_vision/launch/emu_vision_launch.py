#!/usr/bin/env python3
"""
Launch file for emu vision system

Starts the complete vision pipeline including:
- Camera drivers for stereo pair
- Emu observer node for human detection and tracking
- Image processing and visualization nodes

Usage:
    ros2 launch emu_vision emu_vision_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for emu vision system."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('emu_vision')
    config_file = os.path.join(pkg_dir, 'config', 'emu_vision_config.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='/camera',
        description='Camera namespace for stereo pair'
    )
    
    processing_fps_arg = DeclareLaunchArgument(
        'processing_fps',
        default_value='10.0',
        description='Vision processing frame rate (FPS)'
    )
    
    # Emu Observer Node - Main vision processing
    emu_observer_node = Node(
        package='emu_vision',
        executable='emu_observer',
        name='emu_observer',
        namespace='emu',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'processing_fps': LaunchConfiguration('processing_fps'),
            }
        ],
        remappings=[
            ('/camera/left/image_raw', [LaunchConfiguration('camera_namespace'), '/left/image_raw']),
            ('/camera/right/image_raw', [LaunchConfiguration('camera_namespace'), '/right/image_raw']),
            ('/camera/left/camera_info', [LaunchConfiguration('camera_namespace'), '/left/camera_info']),
            ('/camera/right/camera_info', [LaunchConfiguration('camera_namespace'), '/right/camera_info']),
        ],
        output='screen'
    )
    
    # Camera driver nodes (placeholder - would be replaced with actual camera drivers)
    left_camera_node = Node(
        package='v4l2_camera',  # or appropriate camera driver package
        executable='v4l2_camera_node',
        name='left_camera',
        namespace=LaunchConfiguration('camera_namespace'),
        parameters=[
            {
                'video_device': '/dev/video0',
                'image_size': [1920, 1080],
                'camera_frame_id': 'left_camera_frame',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('image_raw', 'left/image_raw'),
            ('camera_info', 'left/camera_info'),
        ],
        output='screen'
    )
    
    right_camera_node = Node(
        package='v4l2_camera',  # or appropriate camera driver package  
        executable='v4l2_camera_node',
        name='right_camera',
        namespace=LaunchConfiguration('camera_namespace'),
        parameters=[
            {
                'video_device': '/dev/video1',
                'image_size': [1920, 1080],
                'camera_frame_id': 'right_camera_frame',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('image_raw', 'right/image_raw'),
            ('camera_info', 'right/camera_info'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        camera_namespace_arg,
        processing_fps_arg,
        emu_observer_node,
        # Uncomment when camera drivers are available
        # left_camera_node,
        # right_camera_node,
    ])