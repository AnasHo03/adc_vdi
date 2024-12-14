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


    
    # Lane recognition
    ampel = Node(
        package='ampel',
        executable='ampel',
        name='ampel',
        output='screen',
    )

    # Line follower
    schilder = Node(
        package='schilder',
        executable='schilder',
        name='schilder',
        output='screen',
    )

    # Micro-ROS Agent
    new_lane_recognition = Node(
        package='lane_recognition',
        executable='new_lane_recognition',
        name='new_lane_recognition',
        output='screen',
    )
    
    return LaunchDescription([
        new_lane_recognition,
        ampel,
        schilder,
    ])

