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
  dynamic_reconfigure
)

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/RVizInteractiveMarker.cfg
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_marker src/simple_marker_node.cpp src/simple_marker.cpp)
target_link_libraries(simple_marker ${catkin_LIBRARIES})

add_executable(basic_control src/basic_control_node.cpp)
target_link_libraries(basic_control ${catkin_LIBRARIES})

add_executable(display_interactive_markers src/display_int_markers_node.cpp src/display_int_markers.cpp)
add_dependencies(display_interactive_markers ${PROJECT_NAME}_gencfg)
target_link_libraries(display_interactive_markers ${catkin_LIBRARIES})