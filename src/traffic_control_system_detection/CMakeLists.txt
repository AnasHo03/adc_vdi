cmake_minimum_required(VERSION 3.5)
project(traffic_control_system_detection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ultralytics REQUIRED)

# Set the path to your Python script
set(script_files 
  "scripts/image_processor_only_opencv.py"
  "scripts/image_processor_tensorrt.py"
  "scripts/publishing_usbimg.py"
  "scripts/listener_opencv.py"
  "scripts/traffic_light.py"
)

# Install the Python script
install(PROGRAMS ${script_files}
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the model file
install(DIRECTORY model_files
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
