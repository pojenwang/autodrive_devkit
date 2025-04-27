cmake_minimum_required(VERSION 3.5)
project(smart_gap_follow)

# Enable C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include headers
include_directories(include)

# Build an executable instead of a component
add_executable(smart_gap_follow_node
  src/smart_gap_follow_node/smart_gap_follow_node.cpp
)

# Link libraries
ament_target_dependencies(smart_gap_follow_node
  rclcpp
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  std_msgs
)

# Install the executable
install(TARGETS
  smart_gap_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
