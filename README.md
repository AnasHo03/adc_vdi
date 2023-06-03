# adc
### Package Overview 
All packages including drivers, submodules, and dependencies are available inside the src directory. Our main modules are the following ROS packages:

    ├── depth_from_rgb      : Package that calculates distance matching between the RGB image with the depth image
    ├── lane_recognition    : Package to recognize lane lines based on image processing algorithms
    ├── line_follower       : Package that outputs steering command velocity based on the offset from the lane midpoint
    ├── team_interfaces           : Custom messages definition for our autonomous racing and parking module
    ├── vehicle_control     : Package to control the vehicle using the ackermann message and VESC motor  
    ├── zed_perception      : Package to launch the ZED camera nodes, including config for the ZED object detection API

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
Open a new terminal and run the following command to launch VESC driver:

    docker run --runtime nvidia -it --rm --network host --privileged \
    -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws \
    stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
    bash -c "cd /home/workspaces/ros2_ws && \
    source install/setup.bash && \ 
    ros2 launch vesc_driver vesc_driver_node.launch.py"
    
 Open a new terminal and run the manual vehicle control:

    docker run --runtime nvidia -it --rm --network host --privileged \
    -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws \
    stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
    bash -c "cd /home/workspaces/ros2_ws && \
    source install/setup.bash && \ 
    ros2 launch vehicle_control manual_control.launch.py"
    
 ### TODO
- Test the above two launch files
- Simplify by launching VESC driver inside the mmanual control launch file
- Work on getting STM32 Ultrasonic sensor data using MicroROS
