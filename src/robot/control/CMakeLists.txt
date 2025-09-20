cmake_minimum_required(VERSION 3.10)
project(control)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build library
add_library(control_lib
  src/control_core.cpp)
target_include_directories(control_lib
  PUBLIC include)
ament_target_dependencies(control_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# Build node executable
add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node control_lib)
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME})

# Install config and launch files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
