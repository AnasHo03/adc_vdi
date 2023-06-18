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

    # ackermann_to_vesc
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node.launch.py',
        name='ackermann_to_vesc',
        output='screen',
    )

    return LaunchDescription([
        vesc_driver_node,
        twist_to_ackermann_node,
        ackermann_to_vesc_node
    ])

