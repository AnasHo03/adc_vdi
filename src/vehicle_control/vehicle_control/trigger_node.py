#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')

        self.publisher_30hz = self.create_publisher(Bool, '/trigger_30hz', 10)
        self.publisher_4hz = self.create_publisher(Bool, '/trigger_4hz', 10)

        self.timer_30hz = self.create_timer(1.0 / 30.0, self.timer_callback_30hz)
        self.timer_4hz = self.create_timer(1.0 / 4.0, self.timer_callback_4hz)

    def timer_callback_30hz(self):
        msg = Bool()
        msg.data = True  # Placeholder value
        self.publisher_30hz.publish(msg)

    def timer_callback_4hz(self):
        msg = Bool()
        msg.data = True  # Placeholder value
        self.publisher_4hz.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
