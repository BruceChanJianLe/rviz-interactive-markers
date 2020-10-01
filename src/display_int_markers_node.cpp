#include "rviz-interactive-markers/display_int_markers.hpp"

const std::string RosNodeName = "interactive_markers_demo";


int main(int argc, char ** argv)
{
    // Initialize rosnode
    ros::init(argc, argv, RosNodeName);

    // Instantiate display marker object
    interactive_markers_ns::interactive_marker node("interactive_markers_server");

    // Run node
    node.start();

    return 0;
}