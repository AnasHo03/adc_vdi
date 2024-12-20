cmake_minimum_required(VERSION 3.8)
project(vehicle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(team_interfaces REQUIRED)


# Include directories
include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files and directories.
install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}
        )
        
# Install Python scripts
install(PROGRAMS 
  ${PROJECT_NAME}/joy_to_ackermann.py
  ${PROJECT_NAME}/rc_to_joy.py
  ${PROJECT_NAME}/ackermann_to_vesc.py
  ${PROJECT_NAME}/emergency_stop_publisher.py
  ${PROJECT_NAME}/trigger_node.py
  ${PROJECT_NAME}/cross_parking_out.py
  ${PROJECT_NAME}/parrallel_parking_out.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
