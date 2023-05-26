# adc

### Docker development
Clone this repository and recurse submodules
    
    git clone --recursive https://github.com/CryptoHypnos/adc.git

Pull docker image

    docker pull stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0

Example launching ZED camera inside docker container

    docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 bash -c "cd /home/workspaces/ros2_ws && \
    . /home/ros2_ws/install/setup.bash && . install/setup.bash && ros2 launch zed_perception zed_launch.py"

Example launching lidar inside docker container. Make sure that the serial_baudrate is correct.

    docker run --runtime nvidia -it --rm --network host --privileged \
    -v /dev:/dev -v ~/adc:/home/workspaces/ros2_ws \
    stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
    bash -c "cd /home/workspaces/ros2_ws && \
    colcon build && source install/setup.bash && \ 
    ros2 launch sllidar_ros2 sllidar_launch.py"

### Lane Detection
* Inspired by:
<https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/>
* Algorithim
  * Thresholding
  * Bird's eye view
  * Histogram: locate areas of the image that have high concentrations of white pixels
  * Sliding window: returns a polynomial fitting for right and left lines
  * Fill in the lane lines: fill gaps between lines
  * Overlay lane lines on original image
  * Calculate curvatre: return radius of imaginary circle completing left and right lines
  * Calculate car position: return offset from center of two lines


#### lane.py Parameter Tweaking
* Sobel Thresholding: adjust the original value 80

`_, s_binary = edge.threshold(s_channel, (80, 255))`

* ROI: adjust from constructor

`self.roi_points = np.float32`

#### TODO
* Tweak position
* Synthesize a second line in case original is not in frame
