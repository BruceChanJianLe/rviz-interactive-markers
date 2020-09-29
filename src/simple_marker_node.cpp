#include "rviz-interactive-markers/simple_marker.hpp"

const std::string RosNodeName = "simple_marker_node";

int main(int argc, char ** argv)
{
    ros::init(argc, argv, RosNodeName);

    simple_marker node("simple_marker");

    node.start();

    return 0;
}