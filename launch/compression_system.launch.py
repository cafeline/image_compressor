from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('binary_image_compressor')
    param_file = os.path.join(package_dir, 'config', 'publisher_params.yaml')

    if not os.path.exists(param_file):
        print(f"警告: パラメータファイルが見つかりません: {param_file}")

    return LaunchDescription([
        # パブリッシャーノード
        Node(
            package='binary_image_compressor',
            executable='compressed_image_publisher',
            name='compressed_image_publisher',
            parameters=[param_file],
            output='screen',
        ),

        # ビューアーノード
        Node(
            package='binary_image_compressor',
            executable='compressed_image_viewer',
            name='compressed_image_viewer',
            output='screen',
        )
    ])