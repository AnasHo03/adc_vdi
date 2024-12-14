#!/usr/bin/env python3

import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.node import Node
import time
import getch
from std_msgs.msg import UInt8, Int16MultiArray , UInt16MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Define a compatible QoS profile
qos_profile = QoSProfile(
    depth=10
)

class TeleopNode:
    def __init__(self):
        self.node = rclpy.create_node('teleop_node')
        self.publisher = self.node.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        self.lights_pub = self.node.create_publisher(UInt16MultiArray, '/lights', 10)
        self.brake_light_pub = self.node.create_publisher(UInt16MultiArray, '/brake_light', 10)
        self.ackermann_msg = AckermannDrive()
        self.node.get_logger().info("please press the control keys !")

    def distance_callback(self,msg):
        self.distance = msg
        print("hi")
        self.get_logger().info("got message")
    def teleop_control(self):
        speed = 0.0
        steering_angle = 0.0
        STEERING_BIAS = -0.021

        while rclpy.ok():
            key = getch.getch()
            if key == 'w':
                speed = 0.45
                steering_angle =0.0
                self.lights("off","off","run")

            elif key =='s':
                speed = -0.4
                steering_angle =0.0
                self.lights("off","off","parked")

            elif key =='d':
                speed = 0.0
                steering_angle =0.5
            elif key =='a':
                speed = 0.0
                steering_angle = -0.5
            elif key =='e':
                speed = 0.4
                steering_angle =0.5
            elif key =='q':
                speed = 0.4
                steering_angle = -0.5
            elif key =='c':
                speed = -0.4
                steering_angle =0.5
            elif key =='z':
                speed = -0.4
                steering_angle =-0.5
            elif key =='y':
                speed = -0.4
                steering_angle =-0.5  
            elif key =='l':
                self.lights("ON","left","run")
             
            else:
                speed = 0.0
                steering_angle = 0.0
                self.lights("off","off","off")

    
        

            self.ackermann_msg.speed = speed
            self.ackermann_msg.steering_angle = steering_angle + STEERING_BIAS

            self.publisher.publish(self.ackermann_msg)
            
            self.node.get_logger().info(f"Speed: {speed}, Steering Angle: {steering_angle}")

            #rclpy.spin_once(self.node)

    def lights(self,front_light,blinker,brake):
        light_msg= UInt16MultiArray()
        light_msg.data = [0,0,0,0,0,0,0,0,0]
        brake_light_msg = UInt16MultiArray()
        brake_light_msg.data =[0,0,0,0]
        if front_light == "ON" :
            light_msg.data[0] =2
        elif front_light == "on" :
            light_msg.data[0]=1
        elif front_light == "off":
            light_msg.data[0]=0

        if blinker == "right" :
            light_msg.data[4]=1
        elif blinker == "left" :
            light_msg.data[4]=2
            light_msg.data[5]=5
            light_msg.data[6]=5
            light_msg.data[7]=20
        elif blinker == "all" :
            light_msg.data[4]=3
        elif blinker == "off":
            light_msg.data[4]=0
        if brake == "run" :
            brake_light_msg.data[2]=1
        elif brake == "stop" :
            brake_light_msg.data[0]=1
        elif brake == "parked" :
            brake_light_msg.data[1]=1
        elif brake == "off":
            brake_light_msg.data[1]=0
            brake_light_msg.data[2]=0

        self.lights_pub.publish(light_msg)
        self.brake_light_pub.publish(brake_light_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    try:
        teleop_node.teleop_control()
        teleop_node.distance_callback()
    except KeyboardInterrupt:
        pass

    #teleop_node.node.destroy_node()
    #rclpy.shutdown()
    rclpy.spin(teleop_node)

    

if __name__ == '__main__':
    main()
