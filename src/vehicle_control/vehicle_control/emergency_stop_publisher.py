#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from team_interfaces.msg import Emergency  # Assuming you have defined the Emergency message

import getch

class EmergencyStopPublisher(Node):
    def __init__(self):
        super().__init__('emergency_stop_publisher')
        self.publisher_ = self.create_publisher(Emergency, 'emergency', 10)
        self.space_pressed = False

    def publish_emergency(self):
        msg = Emergency()
        msg.emergency_stop = self.space_pressed
        self.publisher_.publish(msg)

    def run(self):
        while rclpy.ok():
            key = getch.getch()
            if key == ' ':
                self.space_pressed = True
                self.get_logger().info('Publishing emergency stop once!')
                self.publish_emergency()
            if key == 'r':
                self.space_pressed = False
                self.get_logger().info('Publishing resume once!')
                self.publish_emergency()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    # Always publish True when exiting the node
    node.space_pressed = True
    node.publish_emergency()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
