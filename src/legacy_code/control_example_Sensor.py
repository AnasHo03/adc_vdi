import rospy
import numpy as np
from rospy import Timer as _timer
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_ctrl.msg import vescCtrl
from std_msgs.msg import UInt8
from sensor_msgs.msg import Range
import message_filters
import time
import subprocess
import sys
	
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
        self.rpm_target = -900
        self.str_target = MAX_RAD

        self.DIAMETER = 0.097
        self.GEAR_RATIO = 8.95
        self.MISTERIOUS_FACTOR = 2 

        self.shutdowned = False
        self.started = False


        # init controller
        self.ctrl = Pure_Pursuit_Controller(self.l_f, self.l_r)

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

        # caches
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

    def rc_callback(self, msg):
        if self.started == True:
            if msg.data == 2:
                rospy.signal_shutdown('Killswitch activated!')

        else:
            if msg.data == 0:
                self.started = True

    def timer_cb(self, times):
        global drv_dir, drv_chg

        # only start if start command received
        if self.started == False: return
        
        usb = np.zeros(3) #(cache_usb2,cache_usb3)
        usf = np.zeros(3) #(cache_usf2,cache_usf3,cache_usf4)
        us_thresh_f = 0.08 # [m]
        us_thresh_b = 0.10 # [m]

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

        # change driving direction
        if drv_dir == -1:
            if np.any(usb < us_thresh_b):
                drv_chg = True
            else:
                drv_chg = False
        else:
            if np.any(usf < us_thresh_f):
                drv_chg = True
            else:
                drv_chg = False

        if np.all(usf > 0.4):
            # Drive left
            self.send_vesc_Ctrl(enableMask, 0, -MAX_RAD, MAX_RAD)
            time.sleep(0.1)
            self.send_vesc_Ctrl(enableMask, 900, -MAX_RAD, MAX_RAD)
            time.sleep(0.7)
            # Drive straight
            self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
            time.sleep(0.1)
            self.send_vesc_Ctrl(enableMask, 900, 0, MAX_RAD)
            time.sleep(1.2)
            # Drive right
            self.send_vesc_Ctrl(enableMask, 0, MAX_RAD, MAX_RAD)
            time.sleep(0.1)
            self.send_vesc_Ctrl(enableMask, 900, MAX_RAD, MAX_RAD)
            time.sleep(2.4)
            # Stop
            self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)

            #self.send_vesc_Ctrl(enableMask, 900, MAX_RAD/2, MAX_RAD)
            #time.sleep(4)
            print("Car ready to enter pit lane -> shut down")
            rospy.signal_shutdown('Enter pitlane')
            

        #sys.exit(exit_code)
        if drv_chg == True:
            self.rpm_target *= -1
            drv_dir *= -1
            self.str_target *= -1
            self.send_vesc_Ctrl(enableMask, 0, self.str_target, MAX_RAD)
            time.sleep(1)



        self.send_vesc_Ctrl(enableMask, self.rpm_target, self.str_target, MAX_RAD)
    
    def spawn_program_and_die(self,program, exit_code=0):
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
        self.spawn_program_and_die(['python3', '/home/nano/mechatroniklabor_ws/src/wettbewerb/Lanecontrol.py'])        

def main():
    print(__file__ + " started !!")

    node = Chatter()
    print("Node initialized")
    rospy.spin()




if __name__ == '__main__':
    main()
