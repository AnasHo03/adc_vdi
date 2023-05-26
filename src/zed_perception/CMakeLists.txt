cmake_minimum_required(VERSION 3.8)
project(zed_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy  REQUIRED)
find_package(zed_interfaces REQUIRED)

set(dependencies
    rclcpp
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files and directories.
install(DIRECTORY
        launch config rviz ${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()