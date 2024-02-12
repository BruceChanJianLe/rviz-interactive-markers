#include "rviz-interactive-markers/int_marker.hpp"

static constexpr auto RosNodeName = "int_marker_node";
int main (int argc, char *argv[])
{
  // Init ROS
  ros::init(argc, argv, RosNodeName);

  // Init int marker
  interaction::IntHandler node;

  // Run node
  node.run();

  // Block from exit
  ros::spin();

  return 0;
}
