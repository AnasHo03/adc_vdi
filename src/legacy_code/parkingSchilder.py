import rospy
import numpy as np
from rospy import Timer as _timer
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_ctrl.msg import vescCtrl
from std_msgs.msg import UInt8
from sensor_msgs.msg import Range
import message_filters
import time
import sys
import subprocess

####################Schilder########################
import sys
import argparse
import os
import pycuda.autoinit
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.append('/home/nano/mechatroniklabor_ws/src/tensorrt_demos')
sys.path.append('/home/nano/mechatroniklabor_ws/src/tensorrt_demos/utils')
sys.path.append('/home/nano/mechatroniklabor_ws/src/tensorrt_demos/plugins')
from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
WINDOW_NAME = 'TrtYOLODemo'
###########################################

#Lane Detection
from Ampel import detectTrafficLight
from sensor_msgs.msg import Image as ROS_Image
import cv2
import ATS 
from Gestrichelt import GestrichelteLinie

#constants DONT Touch !
MAX_RAD = 0.4692 # [rad]	max possible rad for left and right
enableMask = 0x0
enableMask = enableMask | 0x1  # enable rpm
enableMask = enableMask | 0x4  # enabel steering

#constants can be changed
RATE = 100 # [Hz]
TIMER_CB_INTERVALL = 0.1
drv_dir = -1 
drv_chg = False
str_dir = 1

#parking constants
parallel_parking = False #Change to "True" for parallel parking

tolerance=15            #tolerance distance in cm

taken = 27     #distance taken spot in cm
free_parallel = 90    #distance free_parallel in cm
free_cross = 120        #########NEW######distance free_cross in cm
no_parking = 300        #distance no parking in cm

#range_taken = range(0, taken+tolerance)
#range_free_parallel = range(free_parallel-tolerance, free_parallel+tolerance)
#range_free_cross = range(free_cross-tolerance, free_cross+tolerance)
#range_no_parking = range(free_cross+tolerance, no_parking+tolerance)
range_taken = range(0, 45)
range_free_parallel = range(46, 90)
range_free_cross = range(91, 140)
range_no_parking = range(141, 300)

min_US_dist = 3          #US-Sensor minimum dist to stop in cm

dt_thresh = 1         #[s]
x = 300

wait_detection_p = 1.5 #time to wait after Spot detection parallel
wait_detection_c = 1.3 #time to wait after Spot detection cross

usb5_detect_parking = "no_parking"
usf1_detect_parking = "no_parking"

#variables for parrallel Parking
arr_usb5_dist = np.array([x,x,x,x,x])
arr_usf1_dist = np.array([x,x,x,x,x])

###################################################################################################################################

class Chatter():
    def __init__(self):
        
        ########################
        print ('2 Minute Warning')
        args = self.parse_args()
        if args.category_num <= 0:
            raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
        if not os.path.isfile('/home/nano/mechatroniklabor_ws/src/tensorrt_demos/yolo/%s.trt' % args.model):
            raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)
        self.cam = Camera(args)
        if not self.cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')  
        cls_dict = get_cls_dict(args.category_num)
        vis = BBoxVisualization(cls_dict)

        self.trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)
        self.conf_thresh=args.conf_thresh
        print ('Trafficsigns loaded!')
        self.schilderkannt=False
        self.schilderkannt2=True
        #################################

        self.rpm_target = 2000###############2000           #Speed to Drive
        self.str_target = MAX_RAD       

        self.shutdowned = False
        self.killswitch_shutdown = False
        self.last_state_parking = "no_parking"
        self.p_slot_det = False
        self.c_slot_det = False
        self.t0 = 0
        self.started = False
	
        # init lane detection constants
        self.state = "both"
        self.stop = False
        self.servoW = 0
        self.gestrichelt=False####################################

        # init ROS
        rospy.init_node('accel_ctrl', anonymous=False)
        rate = rospy.Rate(RATE)

        # init publisher
        self.ctrl_pub = rospy.Publisher('/vehicle_stack/vesc/ackermann_to_vesc/vesc_ctrl', vescCtrl, queue_size=1) 

        # init Subcriber
        rospy.Subscriber('/vehicle_stack/rc/rc_receiver/rc_mode', UInt8, self.rc_callback) 
        rospy.Subscriber('/camera/color/image_raw', ROS_Image, self.cam_callback)

        # caches vor US-Sensors
        self.cache_usb1 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus1', Range), 1)
        self.cache_usb2 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus2', Range), 1)
        self.cache_usb3 = message_filters.Cache(message_filters.Subscriber('/sensor_stack/us_back/backus3', Range), 1)
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
        rospy.Timer(rospy.Duration(TIMER_CB_INTERVALL), self.timer_cb)





    def parse_args(self):
        """Parse input arguments."""
        desc = ('Capture and display live camera video, while doing '
                'real-time object detection with TensorRT optimized '
                'YOLO model on Jetson')
        parser = argparse.ArgumentParser(description=desc)
        parser = add_camera_args(parser)

        parser.add_argument(
            '-c', '--category_num', type=int, default=80,
            help='number of object categories [80]')
        parser.add_argument(
            '-t', '--conf_thresh', type=float, default=0.8,
            help='set the detection confidence threshold')
        parser.add_argument(
            '-m', '--model', type=str, default='custom-yolov4-tiny',
            help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
                'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
                '{dimension} could be either a single number (e.g. '
                '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
        parser.add_argument(
            '-l', '--letter_box', action='store_true',
            help='inference with letterboxed image [False]')
        self.args = parser.parse_args()
        return self.args



	

    def rc_callback(self, msg):
        if self.started == True:
            if msg.data == 2:
                self.killswitch_shutdown = True
                rospy.signal_shutdown('Killswitch activated!')

        else:
            if msg.data == 0:
                self.started = True
    
    def cam_callback(self, col_img_raw):

        # only start if start command received
        if self.started == False: return


        if self.schilderkannt == False:
            print('schild?')
            self.schilderkannt==True
            img = np.frombuffer(col_img_raw.data, dtype=np.uint8).reshape(col_img_raw.height, col_img_raw.width, -1) 
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/1.png",img)      
            img = self.cam.read()
            self.boxes, self.confs, self.clss = self.trt_yolo.detect(img, self.conf_thresh)
            print(self.clss)
            print(self.confs)
            time.sleep(10)
            self.schilderkannt2==True

        if self.schilderkannt2 == True:
            print('Start')
            # Bild einlesen
            img = np.frombuffer(col_img_raw.data, dtype=np.uint8).reshape(col_img_raw.height, col_img_raw.width, -1) 
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
            # Linienerkennung
            self.servoW, self.state = ATS.LaneDetection(img,MAX_RAD,self.servoW, self.state)


            # Einfahrt Boxengasse erkennen
            if self.gestrichelt==False:
                self.strich=GestrichelteLinie(img)
        
            if self.gestrichelt==False and self.strich==True:

                print('gestrichelt gefunden')
                #self.rpm_target = 0

                self.servoW=0
                self.servoW = MAX_RAD * self.servoW/100
                self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)
                time.sleep(0.7)            
                self.servoW=70
                self.servoW = MAX_RAD * self.servoW/100
                self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)
                time.sleep(1.5)
                self.servoW=0
                self.servoW = MAX_RAD * self.servoW/100
                self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)
                time.sleep(1.6)
                self.servoW=-70
                self.servoW = MAX_RAD * self.servoW/100
                self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)
                time.sleep(1.1)

                #self.rpm_target = 0            
                self.rpm_target = 900 ####################900
                self.gestrichelt=True
                print("start parking")



            if self.p_slot_det == False and self.c_slot_det == False:
                self.send_vesc_Ctrl(enableMask, self.rpm_target, self.servoW, MAX_RAD)

    def parking(self, cache_usb5, cache_usf1):
           
        #Write new values into Array
        arr_usb5_dist[0:4]= arr_usb5_dist[1:5]
        arr_usb5_dist[4]=cache_usb5
        arr_usf1_dist [0:4]= arr_usf1_dist[1:5]
        arr_usf1_dist[4]= cache_usf1
        
        #Median of Array (5 last US values)
        dist_usb5 = np.median(arr_usb5_dist) 
        dist_usf1 = np.median(arr_usf1_dist)

        #If clauses detect parking spot (side sensor right back)
        if dist_usb5 in range_no_parking:
            usb5_detect_parking = "no_parking"

        elif dist_usb5 in range_free_parallel:
            usb5_detect_parking = "free_parallel"

        elif dist_usb5 in range_free_cross:
            usb5_detect_parking = "free_cross"

        elif dist_usb5 in range_taken:
            usb5_detect_parking = "taken"

        else:
            print("error usb5")
            usb5_detect_parking = self.last_state_parking
        #If clauses detect parking spot (side sensor right front)
        print(dist_usb5)
        if dist_usf1 in range_no_parking:
            usf1_detect_parking = "no_parking"

        elif dist_usf1 in range_free_parallel:
            usf1_detect_parking = "free_parallel"

        elif dist_usf1 in range_free_cross:
            usf1_detect_parking = "free_cross"

        elif dist_usf1 in range_taken:
            usf1_detect_parking = "taken"

        else:
            print("error usf1")
            usf1_detect_parking = self.last_state_parking
        #Comparison side sensor right front and back
        # have both sensors the same state?
        print("usb5_detect_parking:", usb5_detect_parking)
        if usf1_detect_parking == usb5_detect_parking:
            if parallel_parking==True:
            
                if usb5_detect_parking == "free_parallel":
                    if self.t0 == 0:
                      self.t0 = time.time()
                if (self.last_state_parking == "free_parallel") and (usb5_detect_parking != "free_parallel"):  #change detected?
                    if time.time()-self.t0 > dt_thresh:
                        self.p_slot_det = True
                        self.send_vesc_Ctrl(enableMask, self.rpm_target, -MAX_RAD/2, MAX_RAD)
                    #waiting time after detection
                        time.sleep(wait_detection_p)
                        #self.p_slot_det = True
                    else:
                        self.t0 = 0
                self.last_state_parking = usb5_detect_parking
        
            if parallel_parking==False:   
                if usb5_detect_parking == "free_cross":
                    if self.t0 == 0:
                        self.t0 = time.time()
                if (self.last_state_parking == "free_cross") and (usb5_detect_parking != "free_cross"):  #change detected?
                    if time.time()-self.t0 > dt_thresh:
                        #waiting time after detection
                        #time.sleep(wait_detection_c)
                        self.c_slot_det = True
                    else:
                        self.t0 = 0

                self.last_state_parking = usb5_detect_parking
        
        #print("self.p_slot_det:"+  str(self.p_slot_det))
        #print("usb5_detect_parking:", usb5_detect_parking)

    def timer_cb(self, times):
        # only start if start command received
        #print("Timer cb")
        if self.started == False: return
        
        timestamp = self.cache_usb5.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        cache_usb5 = self.cache_usb5.getElemAfterTime(timestamp).range

        timestamp = self.cache_usf1.getLatestTime()  # !! proof, if the timestamps are realy (almost) the same
        if timestamp == None: return
        cache_usf1 = self.cache_usf1.getElemAfterTime(timestamp).range
        
        if self.gestrichelt == True:
            print("Parkplatzsuche")
            self.parking(cache_usb5*100, cache_usf1*100)

        if self.p_slot_det == True or self.c_slot_det == True:
            self.send_vesc_Ctrl(enableMask, self.rpm_target, -MAX_RAD, MAX_RAD)
            time.sleep(wait_detection_c)
            self.rpm_target = 0
            self.send_vesc_Ctrl(enableMask, self.rpm_target, 0, MAX_RAD)
            print("Car ready for park in.")
            
            rospy.signal_shutdown('Car ready for park in.')




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
        
        # execute next script
        if self.killswitch_shutdown == False:
            if self.p_slot_det == True:
                print("Starting parallel Parking")
                spawn_program_and_die(['python3', '/home/nano/mechatroniklabor_ws/src/wettbewerb/parkingParallelRight_hardcode.py'])
            if self.c_slot_det == True:
                print("Start cross parking")
                spawn_program_and_die(['python3', '/home/nano/mechatroniklabor_ws/src/wettbewerb/parkingCrossRight_hardcode.py'])

def spawn_program_and_die(program, exit_code=0):
    """
    Start an external program and exit the script 
    with the specified return code.

    Takes the parameter program, which is a list 
    that corresponds to the argv of your command.
    """
    # Start the external program
    subprocess.Popen(program)
    # We have started the program, and can suspend this interpreter
    sys.exit(exit_code)

def main():
    print(__file__ + " started !!")

    node = Chatter()
    print("Node initialized")
    rospy.spin()




if __name__ == '__main__':
    main()
