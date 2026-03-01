cmake_minimum_required(VERSION 3.5)
project(fyp_nav2)

# Default to C++17 (for Nav2 dependencies if building C++ code later)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------------
# INSTALL MAPS / PARAMS / LAUNCH FILES
# --------------------------------------------------------------------------------

# Install maps (my_map.yaml & .pgm)
install(
  DIRECTORY
    maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Nav2 parameter files
install(
  DIRECTORY
    params
  DESTINATION share/${PROJECT_NAME}
)

# Install behavior trees
install(
  DIRECTORY
    behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz configuration files
install(
  DIRECTORY
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------
# EXPORT PACKAGE
# --------------------------------------------------------------------------------

ament_package()
