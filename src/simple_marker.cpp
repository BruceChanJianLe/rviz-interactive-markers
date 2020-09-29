#include "rviz-interactive-markers/simple_marker.hpp"


simple_marker::simple_marker()
{
    ;
}


simple_marker::~simple_marker()
{
    ;
}


simple_marker::simple_marker(std::string server_name)
:   private_nh_("~"),
    server_name_(server_name)
{
    // Load parameters
    private_nh_.param("rate", rate_, 5);
    private_nh_.param<std::string>("frame_id", frame_id_, "map");
}


void simple_marker::start()
{
    // Set process rate
    ros::Rate r(rate_);

    // Instantiate interactive marker server on topic server_name_
    interactive_markers::InteractiveMarkerServer server(server_name_);

    // Define interactive int_marker_msg_
    int_marker_msg_.header.frame_id = frame_id_;
    int_marker_msg_.header.stamp = ros::Time::now();
    int_marker_msg_.name = "simple_marker";
    int_marker_msg_.description = "Simple 1-DOF Control";

    // Define marker_msg_
    marker_msg_.type = visualization_msgs::Marker::CUBE;
    marker_msg_.scale.x = 0.45;
    marker_msg_.scale.y = 0.45;
    marker_msg_.scale.z = 0.45;
    marker_msg_.color.r = 0.5;
    marker_msg_.color.g = 0.5;
    marker_msg_.color.b = 0.5;
    marker_msg_.color.a = 1.0;

    // Define visualization for interactive marker
    viz_int_marker_con_msg_.always_visible = true;
    viz_int_marker_con_msg_.markers.emplace_back(marker_msg_);

    // Add visualization to interactive marker msg
    int_marker_msg_.controls.emplace_back(viz_int_marker_con_msg_);

    // Define control for interactive marker
    con_int_marker_msg_.name = "move_x";
    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    // Add control to interactive marker msg
    int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

    // Insert interactive marker msg t server
    server.insert(int_marker_msg_,
    [](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
    {
        ROS_INFO_STREAM("\n" << feedback->marker_name << 
                        " is now at " << feedback->pose.position.x <<
                        ", " << feedback->pose.position.y <<
                        ", " << feedback->pose.position.z
                        );
    });

    // 'commit' changes and send to all clients
    server.applyChanges();

    while (private_nh_.ok())
    {
        // Allow callbacks
        ros::spinOnce();
        // Let node sleep
        r.sleep();
    }
}