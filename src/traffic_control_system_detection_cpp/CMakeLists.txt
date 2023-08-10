cmake_minimum_required(VERSION 3.5)
project(traffic_control_system_detection_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Set the path to your Python scripts
set(python_script_files 
  "scripts/publishing_usbimg.py"
  "scripts/listener_opencv.py"
  "scripts/traffic_light.py"
)

# Install the Python scripts
install(PROGRAMS ${python_script_files}
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

# Set the include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Add the new C++ source file
set(cpp_source_files
  "scripts/image_processor_only_opencv.cpp"
)

# Create the C++ executable
add_executable(image_processor_only_opencv ${cpp_source_files})

# Link the C++ executable with required libraries
ament_target_dependencies(image_processor_only_opencv
  cv_bridge
  image_transport
  sensor_msgs
  rclcpp
  ${OpenCV_LIBRARIES}
)

# Install the C++ executable
install(TARGETS
  image_processor_only_opencv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
