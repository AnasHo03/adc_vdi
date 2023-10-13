import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    control_config_file = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'control_config.yaml'
    )
    with open(control_config_file, 'r') as file:
        control_config = yaml.safe_load(file)['vehicle_control']['ros_parameters']

    # vesc
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[control_config]
    )

    # twist_to_ackermann
    twist_to_ackermann_node = Node(
        package='twist_to_ackermann',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        parameters=[{'use_stamps': True}],
        output='screen'
    )

    # ackermann_to_vesc from f1tenth
    # ackermann_to_vesc_node = Node(
    #     package='vesc_ackermann',
    #     executable='ackermann_to_vesc_node.launch.py',
    #     name='ackermann_to_vesc',
    #     output='screen',
    # )

    # ackermann_to_vesc from william
    ackermann_to_vesc_node = Node(
        package='vehicle_control',
        executable='ackermann_to_vesc.py',
        name='ackermann_to_vesc',
        output='screen',
        parameters=[control_config]
    )
    
    # Lane recognition
    lane_recognition_node = Node(
        package='lane_recognition',
        executable='lane_recognition',
        name='lane_recognition',
        output='screen',
    )

    # Line follower
    line_follower_node = Node(
        package='line_follower',
        executable='line_follower',
        name='line_follower',
        output='screen',
    )

    # Micro-ROS Agent
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '-b', '921600', '--dev', '/dev/stm32_nucleo'],
        output='screen',
    )
    
    return LaunchDescription([
        micro_ros_agent_node,
        vesc_driver_node,
        # twist_to_ackermann_node,
        ackermann_to_vesc_node,
        #lane_recognition_node,
        #line_follower_node
    ])

