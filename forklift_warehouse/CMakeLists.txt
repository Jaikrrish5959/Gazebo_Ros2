cmake_minimum_required(VERSION 3.8)
project(forklift_warehouse)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/random_world_generator.py
  src/lidar_box_detector.py
  src/reactive_navigator.py
  src/adaptive_box_transport.py
  src/ontology_reasoner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install config files (moveit + rviz)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF overlay files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install ontology files
install(DIRECTORY ontology/
  DESTINATION share/${PROJECT_NAME}/ontology
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
