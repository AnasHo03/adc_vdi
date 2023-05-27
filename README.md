# adc

### Docker development

SSH inside the Car i.e. via VSCode: Ctrl + Shift + P --> Remote SSH: Connect to Host. Terminal --> New Terminal. First time after fresh install:    

Clone this repository and recurse submodules
    
    git clone --recurse-submodules https://github.com/CryptoHypnos/adc.git

Pull docker image (Only do once except dockerfile changes)

    docker pull stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0

##### Example launching ZED camera inside docker container. (Tested working)

Config file can be found in: src/zed_perception/config/zed. Change the camera_model accordingly in zed_launch.py and zed.yaml to 'zed' or 'zed2'.
Open a new terminal and run the following command:

    docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 bash -c "cd /home/workspaces/ros2_ws && \
    . /home/ros2_ws/install/setup.bash && . install/setup.bash && ros2 launch zed_perception zed_launch.py"

Checking whether the topics are published.

    # Opening a new terminal inside docker
    docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0
    ros2 topic list
    # Checking the detected object topics
    ros2 topic echo /zed/zed_node/obj_det/objects 

To visualize in Foxglove

    # If rosbridge not installed
    sudo apt install ros-humble-rosbridge-suite
    # Launching websocket on your PC
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

To visualize ZED camera in RViZ (Only tested PC with ROS 2 Humble + camera connected to it).

    cd src/zed_perception/rviz/ && \
    ros2 run rviz2 rviz2 -d zed2.rviz

##### Example launching lidar inside docker container. (Tested working)
Open a new terminal and run the following command:

    docker run --runtime nvidia -it --rm --network host --privileged \
    -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws \
    stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
    bash -c "cd /home/workspaces/ros2_ws && \
    source install/setup.bash && \ 
    ros2 launch sllidar_ros2 sllidar_launch.py"
