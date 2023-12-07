# adc

### Directories Description
    ├──  ackermann_msgs                      : Needed dependancy (submodule)
    ├──  depth_from_rgb                      : Package that calculates distance matching between the RGB image with the depth image (not used and not tested)
    ├──  frame_samples_zed_troubleshoot/4    : Frames from camera will be writte here (when debugging enabled from lane_recognition)
    ├──  lane_recognition                    : Main perception package. Used to detect lines of lane and classify signs
    ├──  legacy_code                         : Old team's code
    ├──  line_follower                       : Control package containing the node with the routines for time trials and pursuit racing
    ├──  micro_ros_setup                     : Needed dependancy (submodule)
    ├──  mxcarkit_imu_message                : Package for custom messages coming from Nucleo
    ├──  mxcarkit_uss_message                : Package for custom messages coming from Nucleo
    ├──  mxcarkit_vehctrl_message            : Package for custom messages coming from Nucleo
    ├──  nucleo_files                        : Contains latest script for Nucleo with ROS2 support
    ├──  sign_detection/model_files          : Contains file for the best classifier
    ├──  sllidar_ros2                        : Dependancy (submodule). Needed for LIDAR but not used
    ├──  team_interfaces                     : Package for custom messages
    ├──  traffic_control_system_detection    : Perception package for classifier and traffic sign detection
    ├──  transport_drivers                   : Needed dependancy (submodule)
    ├──  udp_msgs                            : Needed dependancy (submodule)
    ├──  udp_msgs                            : Package for micro-ros
    ├──  vehicle_control                     : Package containing bring up launch file, emergency stop node, and parallel parking out node
    ├──  vesc                                : Needed dependancy (submodule)
    ├──  vesc_firmware/config                : Contains config files for the Vesc
    ├──  yolov8_ros                          : Needed dependancy (submodule)
    ├──  Zed-ros2-interfaces                 : Needed dependancy (submodule)
    ├──  Zed-ros2-wrapper                    : Needed dependancy (submodule)
    ├──  zed_perception                      : Needed dependancy (submodule)

    
### Main Packages

    ├── lane_recognition                    : Main perception package.
    ├── line_follower                       : Control package containing the node with the routines for time trials and pursuit racing
    ├── vehicle_control                     : Package containing bring up launch file, emergency stop node, and parallel parking out node
    ├── traffic_control_system_detection    : Perception package for classifier and traffic sign detection

    
### Important Notes!
-Currently the perception module lane_recognition must be run from outside of Docker as the used modules for the classifier only function on the main host machine (Jetson). It is highly recommended to either move all modules to Docker or to move all modules to local machine (Jetson). Due to time cosntraints we were not able to do this.
-Push the reset button on the car after launching the bring-up launch file or when starting the micro-ros node!!!!!!!!!! Failing to do so will result in the car thrusting at maximum speed in case you have line_follower running
-The password for the system is mxck

### Initial Setup
-SSH inside the Car: In VSCode: Ctrl + Shift + P --> Remote SSH: Connect to Host. Terminal --> New Terminal
-Clone this repository and recurse submodules (First time)
    
    git clone --recurse-submodules https://github.com/CryptoHypnos/adc.git

-Pulling new changes

    git pull --recurse-submodules
    git submodule update --init --recursive

-Pulling docker image (Only do once except dockerfile changes). This dockerfile only works for Jetson Linux 35.x.

    docker pull stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0
    
### Bring Up and Operation
0) Prop up the car so that the wheels are not touching either the ground or any cables 
1) SSH inside the Car: In VSCode: Ctrl + Shift + P --> Remote SSH: Connect to Host. Terminal --> New Terminal

2.1) Open a new terminal on local machine and navigate to src

    cd src
2.2) Source packages

    source install/setup.bash
2.3) Run the perception node (xxx is the thresholding for binarization) 

    ros2 run lane_recognition lane_recognition 130
3.1) Open 3 new terminals inside Docker

    docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0

3.2) Go to our directory inside Docker (in each)

    cd /home/workspaces/ros2_ws/
3.3) Source packages (in each)

    source /home/ros2_ws/install/setup.bash && source install/setup.bash
3.4) Launch or run the following
3.4.1) First terminal: bring-up launch file

    ros2 launch vehicle_control autonomous_control.launch.py 
3.4.2) !!!!!!!!!! PUSH THE RESET BUTTON ON THE CAR !!!!!!!!!! Failing to do so will result in the car thrusting at maximum speed

3.4.3) Second terminal: emergency shutdown node

    ros2 run vehicle_control emergency_stop_publisher.py 
3.4.4) Third terminal: (x.x is min speed. y.y is max speed)
    
    ros2 run line_follower line_follower 0.4 0.8

###### Note: Ctrl + D to quit docker environment. Always git pull changes outside docker, otherwise we will sometimes get permission issues with colcon build and git.

   
### Maintainers
Rachid Alhourani: General questions, hardware, line_follower

Jason Sutanto: line_follower, lane_recognition

Johannes Schoch: Classifier

 ### TODO (Suggestions)
 This is a supplemental list to that found in the written report
- Ask Mdynamix for missing screws
- Figure out the problem with the car going at max throttle when the reset button is not pressed
