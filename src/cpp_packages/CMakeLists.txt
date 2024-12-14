cmake_minimum_required(VERSION 3.8)
project(cpp_packages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mxcarkit_uss_message REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
#	"msg/USSCustomMessage.msg"
#)

#include directories
include_directories(
	${rclcpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${cv_bridge_INCLUDE_DIRS}
	${ZED_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	/usr/local/include/opencv4
	/opt/ros/humble/install/include/cv_bridge
)

# Add the executable
add_executable(lane_detector src/lane_detector.cpp)

# link libraries
target_link_libraries(lane_detector
	${rclcpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	${OpenCV_LIBRARIES}
	${cv_bridge_LIBRARIES}
	${ZED_LIBRARIES}
	${CUDA_LIBRARIES}
)
ament_target_dependencies(lane_detector
	rclcpp
	ackermann_msgs
	mxcarkit_uss_message
	std_msgs
	sensor_msgs
)




# Install the executable
install(TARGETS
  lane_detector
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
