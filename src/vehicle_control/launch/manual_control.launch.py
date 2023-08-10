
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

    # Declare arguments
    control_type_arg = DeclareLaunchArgument('control_type', default_value='rc')
    joy_port_arg = DeclareLaunchArgument('joy_port', default_value='/dev/input/js0')

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

    # # rc control
    # rc_to_joy_node = Node(
    #     package='vehicle_control',
    #     executable='rc_to_joy.py',
    #     name='rc_to_joy',
    #     output='screen',
    #     parameters=[control_config],
    #     condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'rc'"]))
    # )

    # # joystick
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     output='screen',
    #     parameters=[{'dev': LaunchConfiguration('joy_port')}],
    #     condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'joy'"]))
    # )

    # # joy_to_ackermann
    # joy_to_ackermann_node = Node(
    #     package='vehicle_control',
    #     executable='joy_to_ackermann.py',
    #     name='joy_to_ackermann',
    #     output='screen',
    #     parameters=[control_config]
    # )

    # ackermann_to_vesc
    ackermann_to_vesc_node = Node(
        package='vehicle_control',
        executable='ackermann_to_vesc.py',
        name='ackermann_to_vesc',
        output='screen',
        parameters=[control_config]
    )

    return LaunchDescription([
        control_type_arg,
        joy_port_arg,

        vesc_driver_node,
        # rc_to_joy_node,
        # joy_node,
        # joy_to_ackermann_node,
        ackermann_to_vesc_node
    ])

