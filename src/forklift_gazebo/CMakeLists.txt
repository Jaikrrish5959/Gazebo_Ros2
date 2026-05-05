cmake_minimum_required(VERSION 3.8)
project(forklift_gazebo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
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

# Find Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER 8)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Build ForkLiftPlugin
add_library(ForkLiftPlugin SHARED plugins/forklift.cc)
target_link_libraries(ForkLiftPlugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
ament_target_dependencies(ForkLiftPlugin
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2_ros
)

# Build ForkLiftWorld
add_library(ForkLiftWorld SHARED plugins/fork_lift_world.cc)
target_link_libraries(ForkLiftWorld
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
ament_target_dependencies(ForkLiftWorld
  rclcpp
)

# Note: Temporarily excluding FreeBody plugins as source code not available in restore

# Install plugin libraries
install(TARGETS
  ForkLiftPlugin
  ForkLiftWorld
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  src/keyboard_teleop.py
  src/auto_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.world"
)

# Install models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf"
)

# Install STL files into models directory
install(DIRECTORY stl/
  DESTINATION share/${PROJECT_NAME}/models/fork_lift_shift/stl
  FILES_MATCHING PATTERN "*.STL"
)

ament_package()