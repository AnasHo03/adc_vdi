from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():

    # Declare arguments
    control_type_arg = DeclareLaunchArgument('control_type', default_value='rc')
    control_config_arg = DeclareLaunchArgument('control_config', default_value='vehicle_control/config/control_config.yaml')
    joy_port_arg = DeclareLaunchArgument('joy_port', default_value='/dev/input/js0')
    vesc_port_arg = DeclareLaunchArgument('vesc_port', default_value='/dev/vesc')

    # Load parameters
    load_params = ComposableNode(
        package='vehicle_control',
        plugin='control_config',
        parameters=[LaunchConfiguration('control_config')],
        name='param_loader'
    )

    # vesc
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[{'port': LaunchConfiguration('vesc_port')}]
    )

    # rc control
    rc_to_joy_node = Node(
        package='vehicle_control',
        executable='rc_to_joy.py',
        name='rc_to_joy',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'rc'"]))
    )

    # joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': LaunchConfiguration('joy_port')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'joy'"]))
    )

    # joy_to_ackermann
    joy_to_ackermann_node = Node(
        package='vehicle_control',
        executable='joy_to_ackermann.py',
        name='joy_to_ackermann',
        output='screen'
    )

    # ackermann_to_vesc
    ackermann_to_vesc_node = Node(
        package='vehicle_control',
        executable='ackermann_to_vesc.py',
        name='ackermann_to_vesc',
        output='screen'
    )

    return LaunchDescription([
        control_type_arg,
        control_config_arg,
        joy_port_arg,
        vesc_port_arg,
        load_params,
        vesc_driver_node,
        rc_to_joy_node,
        joy_node,
        joy_to_ackermann_node,
        ackermann_to_vesc_node
    ])

