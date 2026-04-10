cmake_minimum_required(VERSION 3.8)
project(fyp_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install robot description resources (URDF, meshes, RViz, launch)
# IMPORTANT: must be before ament_package()
install(
  DIRECTORY launch config maps worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS scripts/pose_sync.py scripts/waiter_orchestrator.py scripts/obstacle_gate_node.py scripts/depth_sync_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

