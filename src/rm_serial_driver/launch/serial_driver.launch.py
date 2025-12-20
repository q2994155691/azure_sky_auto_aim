#!/usr/bin/env python3
# launch/serial_driver.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 獲取包的路徑
    pkg_share = get_package_share_directory('rm_serial_driver')
    
    # 參數文件路徑
    default_config = os.path.join(pkg_share, 'config', 'serial_driver.yaml')
    
    # 聲明啟動參數
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the serial driver config file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # 串口驅動節點
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level', 
                      ['serial_driver:=', LaunchConfiguration('debug', default='info')]]
    )
    
    return LaunchDescription([
        config_file_arg,
        debug_arg,
        serial_driver_node
    ])

