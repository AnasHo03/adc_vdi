import rospy
from rospy import Timer as _timer
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_ctrl.msg import vescCtrl
from std_msgs.msg import UInt8
from sensor_msgs.msg import Range
from Ampel import detectTrafficLight

from sensor_msgs.msg import Image as ROS_Image
import time
import numpy as np
import cv2
import ATS
import message_filters

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

#Variables for adjusting Parking
time_stop = 0.5
x = 300
arr_usb1_dist = np.array([x,x,x,x,x])
arr_usf5_dist = np.array([x,x,x,x,x])

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

        self.DIAMETER = 0.097
        self.GEAR_RATIO = 8.95
        self.MISTERIOUS_FACTOR = 2 

        # parking constants
        self.p_stage = 1
        self.i = 0
        self.counter=0

        self.obstacle = False
        self.shutdowned = False
        self.started = False

        # init controller
        self.ctrl = Pure_Pursuit_Controller(self.l_f, self.l_r)
        
        # init lane detection constants
        self.state = "both"
        self.stop = False
        self.servoW = 0
        self.hardcodeFinished = False        

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
        rospy.Timer(rospy.Duration(TIMER_CB_INTERVALL), self.timer_cb)
        rospy.Timer(rospy.Duration(TIMER_CB_INTERVALL), self.security_cb)
        #print('init subscriber')

        # caches
        self.cache_usb1 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus1', Range), 1)
        self.cache_usb2 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus2', Range), 1)
        self.cache_usb3 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus3', Range), 5)
        self.cache_usb4 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus4', Range), 1)
        self.cache_usb5 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus5', Range), 1)
        self.cache_usf1 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_front/frontus1', Range), 1)
        self.cache_usf2 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_front/frontus2', Range), 1)
        self.cache_usf3 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_front/frontus3', Range), 1)
        self.cache_usf4 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_front/frontus4', Range), 1)
        self.cache_usf5 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_front/frontus5', Range), 1)

        # init hook
        rospy.on_shutdown(self.hook)
        self.wait_start_time = time.time() 

        #init time callback
        #rospy.Timer(rospy.Duration(TIMER_CB_INTERVALL), self.timer_cb)

    def security_cb(self,times):
        usf = np.zeros(3) #(cache_usf2,cache_usf3,cache_usf4)
        usl = np.zeros(2)
        us_thresh_l = 0.20 #[m]
        us_thresh_f = 0.10 # [m]
        # read usf
        timestamp = self.cache_usf2.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usf[0] = self.cache_usf2.getElemAfterTime(timestamp).range

        timestamp = self.cache_usf3.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usf[1] = self.cache_usf3.getElemAfterTime(timestamp).range

        timestamp = self.cache_usf4.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usf[2] = self.cache_usf4.getElemAfterTime(timestamp).range

        if np.any(usf < us_thresh_f):
            self.obstacle = True
        else:
            self.obstacle = False
            #print("no obstacle")

        # read usl
        timestamp = self.cache_usf5.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usl[0] = self.cache_usf5.getElemAfterTime(timestamp).range

        timestamp = self.cache_usb1.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usl[1] = self.cache_usb1.getElemAfterTime(timestamp).range

        if np.any(usl < us_thresh_l) and self.hardcodeFinished == True:
            self.obstacle = True
        else:
            self.obstacle = False
            #print("no obstacle")

        

    def rc_callback(self, msg):
        if self.started == True:
            if msg.data == 2:
                rospy.signal_shutdown('Killswitch activated!')

        else:
            if msg.data == 0:
                self.started = True

    def cam_callback(self, col_img_raw):
        #print("cam callback")
        # only start if start command received
        if self.started == False: return

        # only start if hardcode finished
        if self.hardcodeFinished == False: return

        # Bild einlesen
        img = np.frombuffer(col_img_raw.data, dtype=np.uint8).reshape(col_img_raw.height, col_img_raw.width, -1) 
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Linienerkennung
        self.servoW, self.state = ATS.LaneDetection(img,MAX_RAD,self.servoW, self.state)
                
        # Ampelerkennung
        self.stop = detectTrafficLight(img[150:300,:,:],self.stop)
        if self.stop == True:
            self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
        else:
            self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)

        
        
    def timer_cb(self, times):
        #print("timer callback")
        # only start if start command received
        if self.started == False: return

        # only run if hardcode not finished
        if self.hardcodeFinished == True: return

        usb = np.zeros(3) #(cache_usb2,cache_usb3)
        #usf = np.zeros(3) #(cache_usf2,cache_usf3,cache_usf4)
        #us_thresh_f = 0.10 # [m]
        us_thresh_b = 0.25 # [m]

        # read usb
        timestamp = self.cache_usb2.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usb[0] = self.cache_usb2.getElemAfterTime(timestamp).range

        timestamp = self.cache_usb3.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usb[1] = self.cache_usb3.getElemAfterTime(timestamp).range

        timestamp = self.cache_usb4.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        usb[2] = self.cache_usb4.getElemAfterTime(timestamp).range

        # Hardcode deparking
        if self.p_stage == 1:

            print("p_stage: ", self.p_stage)

            # steer straight + drive back
            self.send_vesc_Ctrl(enableMask, 900, 0, MAX_RAD)

            # usb2
            usb_d_list = self.get_us_data(self.cache_usb2)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb2_d = np.min(usb_d_list)

            usb_d_list = self.get_us_data(self.cache_usb3)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb3_d = np.min(usb_d_list)

            usb_d_list = self.get_us_data(self.cache_usb4)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb4_d = np.min(usb_d_list)

            if self.i >=6: 
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                return

            if np.any(usb > us_thresh_b):
                # stop
                print("BACK: ",usb)
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                self.p_stage = 2

        elif self.p_stage == 2:
            print("parking stage: ",self.p_stage)


            if self.counter<11:

            
                if self.obstacle==True:
                    # steer right + stop
                    self.send_vesc_Ctrl(enableMask, 0, MAX_RAD, MAX_RAD)    


                else:
                    # steer right + drive forward
                    self.send_vesc_Ctrl(enableMask, 900, MAX_RAD, MAX_RAD)
                    time.sleep(0.5)
                    self.counter=self.counter+1


            else:

                # steer right + stop
                self.send_vesc_Ctrl(enableMask, 0, MAX_RAD, MAX_RAD)       
                time.sleep(time_stop)

                # steer straight + stop
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)       
                time.sleep(time_stop)

                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                self.hardcodeFinished = True           
                print("Car parked out.")


        
    def get_us_data(self, cache_usb):
        usb_timestamp0 = cache_usb.getOldestTime() 
        usb_timestamp1 = cache_usb.getLatestTime()

        if (usb_timestamp0 == None) or (usb_timestamp1 == None): return
        cache_usb3_msgs = cache_usb.getInterval(usb_timestamp0, usb_timestamp1)

        usb_d_list = []
        for msg in cache_usb3_msgs:
            if msg.range != 3.0:
                usb_d_list.append(msg.range)

        return usb_d_list

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
