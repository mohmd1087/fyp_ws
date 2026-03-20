cmake_minimum_required(VERSION 3.5)
project(fyp_dstar_lite)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package (the fyp_dstar_lite/ module directory)
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the main node script as an executable
install(
  PROGRAMS
    fyp_dstar_lite/dstar_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
