import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージディレクトリを取得
    pkg_dir = get_package_share_directory('binary_image_compressor')
    
    # 引数の宣言
    declare_input_file = DeclareLaunchArgument(
        'input_file',
        default_value=os.path.join(pkg_dir, 'map', 'iscas_museum_map.yaml'),
        description='圧縮する地図ファイルのパス'
    )
    
    declare_block_size = DeclareLaunchArgument(
        'block_size',
        default_value='24',
        description='ブロックサイズ'
    )
    
    declare_iterations = DeclareLaunchArgument(
        'benchmark_iterations',
        default_value='1000000',
        description='ベンチマークの反復回数'
    )
    
    # 圧縮画像パブリッシャーノード
    compressed_image_publisher = Node(
        package='binary_image_compressor',
        executable='compressed_image_publisher',
        name='compressed_image_publisher',
        parameters=[{
            'input_file': LaunchConfiguration('input_file'),
            'block_size': LaunchConfiguration('block_size'),
            'threshold': 128,
        }],
        output='screen'
    )
    
    # ベンチマークノード
    benchmark_node = Node(
        package='binary_image_compressor',
        executable='compressed_map_benchmark',
        name='compressed_map_benchmark',
        parameters=[{
            'map_resolution': 0.05,
            'map_origin_x': -9.461811,
            'map_origin_y': -9.107068,
            'benchmark_iterations': LaunchConfiguration('benchmark_iterations'),
            'random_seed': 42,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_input_file,
        declare_block_size,
        declare_iterations,
        compressed_image_publisher,
        benchmark_node,
    ]) 