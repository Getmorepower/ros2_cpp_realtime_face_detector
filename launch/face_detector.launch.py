from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0', description='USB/Laptop camera index'),
        DeclareLaunchArgument('frame_width', default_value='640', description='Camera frame width'),
        DeclareLaunchArgument('frame_height', default_value='480', description='Camera frame height'),
        DeclareLaunchArgument('fps', default_value='30.0', description='Target capture and inference FPS'),
        DeclareLaunchArgument('show_window', default_value='true', description='Display OpenCV HighGUI window'),

        Node(
            package='ros2_face_detector',
            executable='face_detector_node',
            name='realtime_face_detector_node',
            output='screen',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'show_window': LaunchConfiguration('show_window'),
            }]
        )
    ])