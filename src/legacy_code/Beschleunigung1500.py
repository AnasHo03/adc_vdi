import rospy
from rospy import Timer as _timer
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_ctrl.msg import vescCtrl
from std_msgs.msg import UInt8
from Ampel import detectTrafficLight

from sensor_msgs.msg import Image as ROS_Image
import time
import numpy as np
import cv2
import ATS

from PIL import Image
import os
import re
from tqdm import tqdm_notebook
import matplotlib.pyplot as plt
import sys






#constants DONT Touch !
MAX_RAD = 0.4692 # [rad]	max possible rad for left and right
enableMask = 0x0
enableMask = enableMask | 0x1  # enable rpm
enableMask = enableMask | 0x4  # enabel steering

#constants can be changed
RATE = 100 # [Hz]
TIMER_CB_INTERVALL = 0.1


class Pure_Pursuit_Controller():
    def __init__(self, l_f, l_r):
        self.l_f = l_f
        self.l_r = l_r
        self.l = self.l_f + self.l_r 

    def calc_steer_angle(self, alpha, l_d):
        nominator = 2 * self.l * math.sin(alpha)
        delta_f = math.atan(nominator/l_d)
        return delta_f


class Chatter():
    def __init__(self):
        self.l_f = 0.17 #[cm]
        self.l_r = 0.19 #[cm]
        self.CAM2REAR = 0.22 #[cm]
        #self.COG2REAR = 0.22 #[cm] #?!
        self.rpm_target = 1500 
        self.i=0

        self.DIAMETER = 0.097
        self.GEAR_RATIO = 8.95
        self.MISTERIOUS_FACTOR = 2 

        self.shutdowned = False
        self.started = False

        # init controller
        self.ctrl = Pure_Pursuit_Controller(self.l_f, self.l_r)
        
        # init lane detection constants
        self.state = "both"
        self.stop = False
        self.servoW = 0

	# init  models
        #self.v_model = Single_track_model_kinematic(self.l_f, self.l_r)
        #self.servo_motor = Servo_Motor()
        #self.drive_motor = Drive_Motor(self.DIAMETER, self.GEAR_RATIO, self.MISTERIOUS_FACTOR)
	
        # init ROS
        rospy.init_node('accel_ctrl', anonymous=False)
        rate = rospy.Rate(RATE)

        # init publisher
        self.ctrl_pub = rospy.Publisher('/vehicle_stack/vesc/ackermann_to_vesc/vesc_ctrl', vescCtrl, queue_size=1) 

        # init Subcriber
        rospy.Subscriber('/vehicle_stack/rc/rc_receiver/rc_mode', UInt8, self.rc_callback) 
        rospy.Subscriber('/camera/color/image_raw', ROS_Image, self.cam_callback)
        #print('init subscriber')


        # init hook
        rospy.on_shutdown(self.hook)
        self.wait_start_time = time.time() 

        #init time callback
        #rospy.Timer(rospy.Duration(TIMER_CB_INTERVALL), self.timer_cb)

    def rc_callback(self, msg):
        if self.started == True:
            if msg.data == 2:
                rospy.signal_shutdown('Killswitch activated!')

        else:
            if msg.data == 0:
                self.started = True

    def cam_callback(self, col_img_raw):
        # only start if start command received
        if self.started == False: return

        # Bild einlesen
        img = np.frombuffer(col_img_raw.data, dtype=np.uint8).reshape(col_img_raw.height, col_img_raw.width, -1) 
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Linienerkennung
        self.servoW, self.state = ATS.LaneDetection(img,MAX_RAD,self.servoW, self.state)



        self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)
        
    def timer_cb(self, times):
        # only start if start command received
        if self.started == False: return

        # send test command
        self.send_vesc_Ctrl(enableMask, self.rpm_target, 0.3, MAX_RAD)
        

    def longitudinal_ctrl(self):
        rpm_des = self.rpm_target # [rpm], const. lowest velocity = ca. 0.25 m/s
        if self.FINISHED == True:
            rpm_des = max(0, self.rpm_des_last - 200)
        self.rpm_des_last = rpm_des
        return rpm_des

    def send_vesc_Ctrl(self, enableMask, rpm_des, delta_f, MAX_RAD):
        vescCtrlMsg = self.create_vesc_ctrl_msg(enableMask, rpm_des, delta_f, MAX_RAD)
        if self.shutdowned == False: self.ctrl_pub.publish(vescCtrlMsg)

    def create_vesc_ctrl_msg(self, enableMask, rpm_des, delta_f, MAX_RAD):
        vescCtrlMsg = vescCtrl()
        vescCtrlMsg.enableMask = enableMask
        vescCtrlMsg.rpm = int(rpm_des)
        vescCtrlMsg.servoPos = delta_f / MAX_RAD
        vescCtrlMsg.header.stamp = rospy.Time.now()
        return vescCtrlMsg   

    def hook(self):
        self.shutdowned = True
        t0 = time.time()
        t_close = 1

        while (time.time() - t0) < t_close: 
            vescCtrlMsg = vescCtrl()
            vescCtrlMsg.enableMask = enableMask
            vescCtrlMsg.rpm = int(0)
            vescCtrlMsg.servoPos = 0
            vescCtrlMsg.header.stamp = rospy.Time.now()
            self.ctrl_pub.publish(vescCtrlMsg)
            time.sleep(0.1)
        

def main():
    print(__file__ + " started !!")

    node = Chatter()
    print("Node initialized")
    rospy.spin()




if __name__ == '__main__':
    main()
