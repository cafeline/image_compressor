from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='binary_image_compressor',
            executable='compressed_image_viewer',
            name='compressed_image_viewer',
            output='screen',
        )
    ])