cmake_minimum_required(VERSION 3.8)
project(forklift_ware_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Gazebo Harmonic (gz-sim8)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER 8)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# ---------- Gazebo plugins ----------
add_library(ForkLiftPlugin SHARED plugins/forklift.cc)
target_link_libraries(ForkLiftPlugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
ament_target_dependencies(ForkLiftPlugin
  rclcpp geometry_msgs sensor_msgs std_msgs nav_msgs tf2_ros
)

add_library(ForkLiftWorld SHARED plugins/fork_lift_world.cc)
target_link_libraries(ForkLiftWorld
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
ament_target_dependencies(ForkLiftWorld rclcpp)

# ---------- install rules ----------
# Plugin shared libraries → lib/
install(TARGETS
  ForkLiftPlugin
  ForkLiftWorld
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# World SDF
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.sdf" PATTERN "*.world"
)

# Gazebo model (fork_lift_shift SDF + meshes)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# STL meshes (also available at package root for mesh:// URLs)
install(DIRECTORY stl/
  DESTINATION share/${PROJECT_NAME}/stl
  FILES_MATCHING PATTERN "*.STL"
)

# URDF (for robot_state_publisher)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf"
)

# Python scripts (forklift_ops, keyboard_teleop, auto_drive)
install(PROGRAMS
  scripts/forklift_ops.py
  scripts/keyboard_teleop.py
  scripts/auto_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
