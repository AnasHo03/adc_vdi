import rospy
import numpy as np
from rospy import Timer as _timer
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_ctrl.msg import vescCtrl
from std_msgs.msg import UInt8
from sensor_msgs.msg import Range
import message_filters
import time
import math
import sys
import subprocess

#constants DONT Touch !
MAX_RAD = 0.4692 # [rad]	max possible rad for left and right
enableMask = 0x0
enableMask = enableMask | 0x1  # enable rpm
enableMask = enableMask | 0x4  # enabel steering

#constants can be changed
RATE = 100 # [Hz]
TIMER_CB_INTERVALL = 0.05
drv_dir = -1 
drv_chg = False
str_dir = 1

#Variables for adjusting Parking
time_stop = 0.5

class Chatter():
    def __init__(self):
        self.l_f = 0.17 #[cm]
        self.l_r = 0.19 #[cm]
        self.CAM2REAR = 0.22 #[cm]
        #self.COG2REAR = 0.22 #[cm] #?!
        self.rpm_target = 900
        self.str_target = MAX_RAD

        self.DIAMETER = 0.097
        self.GEAR_RATIO = 8.95
        self.MISTERIOUS_FACTOR = 2 

        self.shutdowned = False

        self.p_stage = 1
        self.i = 0
        self.started = False
	
        # init ROS
        rospy.init_node('accel_ctrl', anonymous=True)
        rate = rospy.Rate(RATE)

        # init publisher
        self.ctrl_pub = rospy.Publisher('/vehicle_stack/vesc/ackermann_to_vesc/vesc_ctrl', vescCtrl, queue_size=1) 

        # init Subcriber
        rospy.Subscriber('/vehicle_stack/rc/rc_receiver/rc_mode', UInt8, self.rc_callback) 

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
        rospy.Timer(rospy.Duration(TIMER_CB_INTERVALL), self.timer_cb)

    def rc_callback(self, msg):
        if self.started == True:
            if msg.data == 2:
                rospy.signal_shutdown('Killswitch activated!')

        else:
            if msg.data == 0:
                self.started = True


    def callback(self, msg):
        pass

    def timer_cb(self, times):
        # only start if start command received
        if self.started == False: return

        usb = np.zeros(3) #(cache_usb2,cache_usb3)
        usf = np.zeros(3) #(cache_usf2,cache_usf3,cache_usf4)
        us_thresh_f = 0.10 # [m]
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
        print("p_stage: ", self.p_stage)

        if self.p_stage == 1:
            #hardcoded stuff
            time.sleep(1)

            # measure distance to car and time for drive back - usf1
            usb_d_list = self.get_us_data(self.cache_usf1)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usf1_d = np.min(usb_d_list)

            usb_d_list = self.get_us_data(self.cache_usb5)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb5_d = np.min(usb_d_list)

            h = usb5_d/math.sin(0.628)
            print("usf1_d: ", usf1_d)


            # steer right + stop
            self.send_vesc_Ctrl(enableMask, 0, MAX_RAD, MAX_RAD)    
            time.sleep(time_stop)

            # steer right + drive back
            self.send_vesc_Ctrl(enableMask, -900, MAX_RAD, MAX_RAD)       
            time.sleep(2.6)

            # steer right + stop
            self.send_vesc_Ctrl(enableMask, 0, MAX_RAD, MAX_RAD)       
            time.sleep(time_stop)

            # steer straight + stop
            self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)       
            time.sleep(time_stop)

            # steer straight + drive back
            self.send_vesc_Ctrl(enableMask, -900, 0, MAX_RAD)  
            adt = h/100
            time.sleep(1.5 + adt)

            # stop
            self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD) 
            time.sleep(time_stop)

            # steer left + stop
            self.send_vesc_Ctrl(enableMask, 0, -MAX_RAD, MAX_RAD) 
            time.sleep(time_stop)


            self.p_stage += 1
        
        elif self.p_stage == 2:


            # usf1
            usb_d_list = self.get_us_data(self.cache_usf1)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usf1_d = np.min(usb_d_list)

            # usb5
            usb_d_list = self.get_us_data(self.cache_usb5)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb5_d = np.min(usb_d_list)

            print("usf1_d: ", usf1_d)
            print("usb5_d: ", usb5_d)
            if (abs(usf1_d - usb5_d) <= 0.01) and (self.i >=3):
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                self.p_stage = 4
                return


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


            if np.any(usb < us_thresh_b):
                # steer left + stop
                self.send_vesc_Ctrl(enableMask, 0, -MAX_RAD, MAX_RAD)
                time.sleep(time_stop) 
                # steer right + stop
                self.send_vesc_Ctrl(enableMask, 0, +MAX_RAD, MAX_RAD) 

                self.p_stage += 1
                self.i += 1

            else:
                # steer left + drive back
                self.send_vesc_Ctrl(enableMask, -900, -MAX_RAD, MAX_RAD) 


        elif self.p_stage == 3:

            # usf1
            usb_d_list = self.get_us_data(self.cache_usf1)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usf1_d = np.min(usb_d_list)

            # usb5
            usb_d_list = self.get_us_data(self.cache_usb5)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb5_d = np.min(usb_d_list)

            print("usf1_d: ", usf1_d)
            print("usb5_d: ", usb5_d)
            if (abs(usf1_d - usb5_d) <= 0.01) and (self.i >= 3):
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                self.p_stage = 4
                return

            #Sensor Values Front
            us_d_list = self.get_us_data(self.cache_usf2)
            if (us_d_list == []) or (us_d_list == None) : return
            usf2_d = np.min(us_d_list)

            us_d_list = self.get_us_data(self.cache_usf3)
            if (us_d_list == []) or (us_d_list == None) : return
            usf3_d = np.min(us_d_list)

            us_d_list = self.get_us_data(self.cache_usf4)
            if (us_d_list == []) or (us_d_list == None) : return
            usf4_d = np.min(us_d_list)


            if self.i >=6: 
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                return


            if np.any(usf < us_thresh_f):       #0.06
                # steer right + stop
                self.send_vesc_Ctrl(enableMask, 0, MAX_RAD, MAX_RAD)
                time.sleep(time_stop) 
                # steer left + drive back
                self.send_vesc_Ctrl(enableMask, 0, -MAX_RAD, MAX_RAD) 

                self.p_stage -= 1
                self.i += 1

            else:
                # steer right + drive forward
                self.send_vesc_Ctrl(enableMask, 900, MAX_RAD, MAX_RAD) 

        elif self.p_stage == 4:
            # usf3
            usb_d_list = self.get_us_data(self.cache_usf3)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usf3_d = np.median(usb_d_list)

            # usb3
            usb_d_list = self.get_us_data(self.cache_usb3)
            if (usb_d_list == []) or (usb_d_list == None) : return
            usb3_d = np.median(usb_d_list)

            dd = usf3_d - usb3_d

            if abs(dd) <= 0.04:
                self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)
                self.p_stage = 5
                return
            else:
                if dd >0:
                    self.send_vesc_Ctrl(enableMask, 900, 0, MAX_RAD)
                else:
                    self.send_vesc_Ctrl(enableMask, -900, 0, MAX_RAD)



        elif self.p_stage == 5:

            self.send_vesc_Ctrl(enableMask, 0, 0, MAX_RAD)           
            print("Car parked.")
            time.sleep(0.1)
            rospy.signal_shutdown('Car parked.')




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

        # execute next script
        spawn_program_and_die(['python3', '/home/nano/mechatroniklabor_ws/src/parking_Y/control_example_Sensor.py'])
        

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
    # let rospy spin
    rospy.spin()
    print("Node initialized")



if __name__ == '__main__':
    main()
