from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('binary_image_compressor')
    param_file = os.path.join(package_dir, 'config', 'compressor_params.yaml')

    # 特に重要：パラメータファイルが存在することを確認
    if not os.path.exists(param_file):
        print(f"警告: パラメータファイルが見つかりません: {param_file}")

    return LaunchDescription([
        Node(
            package='binary_image_compressor',
            executable='compressor_node',
            name='compressor_node',
            parameters=[param_file],
            output='screen',
        )
    ])