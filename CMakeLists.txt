cmake_minimum_required(VERSION 3.0.2)
project(rviz-interactive-markers)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  rospy
  tf
  visualization_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_marker src/simple_marker_node.cpp src/simple_marker.cpp)
target_link_libraries(simple_marker ${catkin_LIBRARIES})