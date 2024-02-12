#include "rviz-interactive-markers/int_marker.hpp"

namespace interaction
{
  IntHandler::IntHandler()
    : nh_{}
    , pnh_{"~"}
    , int_srv_{std::make_shared<interactive_markers::InteractiveMarkerServer>(ros::this_node::getName(), pnh_.getNamespace(), false)}
    , frame_id_{"map"}
    , marker_idx_{0}
  {
    // Load ROS Params
    pnh_.param<std::string>("frame_id", frame_id_, "map");

    addNodeMarker();
    addNodeMarker(); // Add to determine whether other markers will disappear while dragging

    display_loop_ = pnh_.createTimer(ros::Duration{UPDATE_RATE}, [this](const ros::TimerEvent &){ this->displayLoop(); });
  }

  IntHandler::~IntHandler()
  {
  }

  void IntHandler::run()
  {
    ROS_INFO_STREAM("STARTED INT MARKER NODE!");
    int_srv_->applyChanges();
  }

  void IntHandler::addNodeMarker()
  {
    ROS_INFO_STREAM("Add marker " << marker_idx_);
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;

    // Add a control marker
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.orientation.x = 0.0;
    control.orientation.y = 0.5;
    control.orientation.z = 0.0;
    control.orientation.w = 0.5;

    // Add a visualiazation marker
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Add control and visual marker
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    int_marker.name = "marker_" + std::to_string(marker_idx_);
    int_marker.description = "marker_" + std::to_string(marker_idx_);
    int_srv_->insert(int_marker);
    int_srv_->setCallback(
        int_marker.name,
        [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){ this->markerCallback(feedback, marker_idx_); }
      );
    marker_idx_++;
  }

  void IntHandler::addFollowMeNodeMarker()
  {
    ;
  }

  void IntHandler::displayLoop()
  {
    ;
  }

  void IntHandler::markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int marked_idx)
  {
    marker_contexts_.emplace_back();
  }
} // interaction
