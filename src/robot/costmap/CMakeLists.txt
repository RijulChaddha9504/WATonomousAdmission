cmake_minimum_required(VERSION 3.10)
project(costmap)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create ROS2 node executable
add_executable(costmap_node src/costmap_node.cpp)

# Link dependencies
ament_target_dependencies(costmap_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# Install executable
install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config or launch files if you have them
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
