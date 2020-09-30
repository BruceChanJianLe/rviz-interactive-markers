#include "rviz-interactive-markers/simple_marker.hpp"

const std::string RosNodeName = "simple_marker_node";

int main(int argc, char ** argv)
{
    // Initialize rosnode
    ros::init(argc, argv, RosNodeName);

    // Instantiate simple maker object
    simple_marker node("simple_marker");

    // Run node
    node.start();

    return 0;
}