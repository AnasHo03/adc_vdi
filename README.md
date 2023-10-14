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

    
### Docker development

SSH inside the Car. Development in VSCode: Ctrl + Shift + P --> Remote SSH: Connect to Host. Terminal --> New Terminal.

Clone this repository and recurse submodules (First time)
    
    git clone --recurse-submodules https://github.com/CryptoHypnos/adc.git

Pulling new changes

    git pull --recurse-submodules
    git submodule update --init --recursive

Pulling docker image (Only do once except dockerfile changes). This dockerfile only works for Jetson Linux 35.x.

    docker pull stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0

Opening a new terminal inside docker

    docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0

Go to our directory inside docker

    cd /home/workspaces/ros2_ws/
    # Source build packages
    source /home/ros2_ws/install/setup.bash && source install/setup.bash
    # Build your package
    colcon build --packages-select <your_package>

###### Note: Ctrl + D to quit docker environment. Always git pull changes outside docker, otherwise we will sometimes get permission issues with colcon build and git.

#### Example launching ZED camera inside docker container. (Tested working)

Config file can be found in: src/zed_perception/config/zed. Change the camera_model accordingly in zed_launch.py and zed.yaml to 'zed' or 'zed2'.
Open a new terminal and run the following command:

    docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 bash -c "cd /home/workspaces/ros2_ws && \
    . /home/ros2_ws/install/setup.bash && . install/setup.bash && ros2 launch zed_perception zed_launch.py" 

To visualize in Foxglove, install rosbridge suite

    sudo apt install ros-humble-rosbridge-suite

Launch websocket on your PC

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

To visualize ZED camera in RViZ (Only tested PC with ROS 2 Humble + camera connected to it).

    ros2 run rviz2 rviz2 -d src/zed_perception/rviz/zed2.rviz

#### Example launching lidar inside docker container. (Tested working)
Open a new terminal and run the following command:

    docker run --runtime nvidia -it --rm --network host --privileged \
    -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws \
    stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
    bash -c "cd /home/workspaces/ros2_ws && \
    source install/setup.bash && \ 
    ros2 launch sllidar_ros2 sllidar_launch.py"

#### Example launching vehicle control using VESC. (Not Tested)    
 Open a new terminal and run the manual vehicle control including VESC driver node:

    docker run --runtime nvidia -it --rm --network host --privileged \
    -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws \
    stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
    bash -c "cd /home/workspaces/ros2_ws && \
    source install/setup.bash && \ 
    ros2 launch vehicle_control autonomous_control.launch.py"
   

 ### TODO
- Work on getting STM32 Ultrasonic sensor data using MicroROS
