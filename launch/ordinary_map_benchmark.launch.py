#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # パラメータの宣言
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            get_package_share_directory('binary_image_compressor'),
            'map', 'iscas_museum_map.yaml'
        ),
        description='Path to the map file (YAML format)'
    )

    declare_benchmark_iterations = DeclareLaunchArgument(
        'benchmark_iterations',
        default_value='1000000',
        description='Number of benchmark iterations'
    )

    declare_random_seed = DeclareLaunchArgument(
        'random_seed',
        default_value='42',
        description='Random seed for reproducible benchmarks'
    )

    # 地図サーバーノード（通常の占有格子地図をパブリッシュ）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'use_sim_time': False
        }]
    )

    # ライフサイクルマネージャー（地図サーバーを起動）
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 通常地図ベンチマークノード
    ordinary_map_benchmark_node = Node(
        package='binary_image_compressor',
        executable='ordinary_map_benchmark',
        name='ordinary_map_benchmark',
        output='screen',
        parameters=[{
            'benchmark_iterations': LaunchConfiguration('benchmark_iterations'),
            'random_seed': LaunchConfiguration('random_seed')
        }]
    )

    return LaunchDescription([
        declare_map_file,
        declare_benchmark_iterations,
        declare_random_seed,
        map_server_node,
        lifecycle_manager_node,
        ordinary_map_benchmark_node
    ]) 