from platform import node
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, Int16MultiArray
from team_interfaces.msg import Lane
from team_interfaces.msg import Emergency



MAX_STEERING_ANGLE = 0.442  # [rad]
MIN_THRUST = 1.0
t = 0.2
t_1 = 1

self.uss_sensors_sub = self.create_subscription(Int16MultiArray, 'uss_sensors', self.uss_callback, qos_profile=qos_profile)        

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change to BEST_EFFORT
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

uss_data_right == np.full(0,1)
uss_data_front == np.full(3)

if uss_data_right(0) != -1 and uss_data_right != -1 and uss_data_right(0)+uss_data_right<30:
    MAX_STEERING_ANGLE = -MAX_STEERING_ANGLE
    MIN_THRUST = -MIN_THRUST

if uss_data_front(0) < 20 and uss_data_back(0):
    for i in range(5):
        ack_msg.steering_angle = MAX_STEERING_ANGLE
        ack_msg.speed = MIN_THRUST
        time.sleep(t)
        MAX_STEERING_ANGLE = -MAX_STEERING_ANGLE
        MIN_THRUST = -MIN_THRUST


if uss_data_front < 20:
    MIN_THRUST = -MIN_THRUST


if uss_data_front(0) <20 or uss_data_back(0)<20:
    ack_msg.steering_angle = -MAX_STEERING_ANGLE
    ack_msg.speed = MIN_THRUST
    time.sleep(t_1)
    ack_msg.steering_angle = 0
    ack_msg.speed = MIN_THRUST
    time.sleep(t_1)

