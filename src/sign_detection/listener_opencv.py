import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InferenceResultListener(Node):
    def __init__(self):
        super().__init__('inference_result_listener')
        self.subscription = self.create_subscription(
            String,
            'detections_topic',
            self.result_callback,
            10
        )

    def result_callback(self, msg):
        self.get_logger().info('Received inference result:\n%s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    inference_result_listener = InferenceResultListener()
    rclpy.spin(inference_result_listener)
    inference_result_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()