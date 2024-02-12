#include "rviz-interactive-markers/int_marker.hpp"

namespace interaction
{
  IntHandler::IntHandler()
    : nh_{}
    , pnh_{"~"}
    , int_srv_{std::make_shared<interactive_markers::InteractiveMarkerServer>(ros::this_node::getName(), pnh_.getNamespace(), false)}
    , frame_id_{"map"}
    , marker_style_{0}
    , marker_idx_{0}
  {
    // Load ROS Params
    pnh_.param<std::string>("frame_id", frame_id_, "map");
    pnh_.param("marker_style", marker_style_, 2);

    if (marker_style_ > 2 || marker_style_ < 0)
    {
      ROS_WARN_STREAM("Marker style can only be in between 0 - 2");
      marker_style_ = 3;
    }

    switch (marker_style_)
    {
      case 0:
        addNodeMarker();
        addNodeMarker(); // Add to determine whether other markers will disappear while dragging
        break;

      case 1:
        addSplitNodeMarker();
        break;

      case 2:
        addFollowMeNodeMarker();
        break;

      default:
        addFollowMeNodeMarker();
        break;
    }


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
    control.orientation.y = std::sqrt(1.f/2.f);
    control.orientation.z = 0.0;
    control.orientation.w = std::sqrt(1.f/2.f);

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

  void IntHandler::addSplitNodeMarker()
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;

    // Add a control marker
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = false;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.orientation.y = std::sqrt(1.f/2.f);
    control.orientation.w = std::sqrt(1.f/2.f);

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 1.0;
    marker.color.a = 0.0;
    marker.scale.x = 0.45 + 0.1;
    marker.scale.y = 0.45 + 0.1;
    marker.scale.z = 0.45 + 0.1;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;

    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    // Add control marker to server
    int_marker.name = "split_marker_" + std::to_string(marker_idx_);
    int_marker.pose = marker.pose;
    // int_marker.description = "follow_me_" + std::to_string(marker_idx_);
    int_srv_->insert(int_marker,
        [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
        { this->splitMarkerCallback(feedback, 0); }
      );
    marker_contexts_.emplace_back(int_marker.pose.position.x, int_marker.pose.position.y);

    // Insert visual marker
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    control.always_visible = true;

    int_marker.name = "split_marker_" + std::to_string(marker_idx_) + "_display";
    // int_marker.pose.position.x += 1.0;
    // int_marker.pose.position.y += 1.0;

    int_marker.controls.clear();
    control.markers.clear();
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
    int_srv_->insert(int_marker);

    // increment last
    ++marker_idx_;
  }

  void IntHandler::addFollowMeNodeMarker()
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;

    // Add a control marker
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = false;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.orientation.y = std::sqrt(1.f/2.f);
    control.orientation.w = std::sqrt(1.f/2.f);

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 1.0;
    marker.color.a = 0.0;
    marker.scale.x = 0.45 + 0.1;
    marker.scale.y = 0.45 + 0.1;
    marker.scale.z = 0.45 + 0.1;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;

    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    // Add first marker to server
    int_marker.name = "follow_me_" + std::to_string(0);
    int_marker.pose = marker.pose;
    int_srv_->insert(int_marker,
        [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
        { this->followMeMarkerCallback(feedback, 0); }
      );
    marker_contexts_.emplace_back(int_marker.pose.position.x, int_marker.pose.position.y);

    // Add follow marker to server
    int_marker.name = "follow_me_" + std::to_string(1);
    int_marker.pose.position.x += 1.0;
    int_marker.pose.position.y += 1.0;
    int_srv_->insert(int_marker,
        [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
        { this->followMeMarkerCallback(feedback, 1); }
      );
    marker_contexts_.emplace_back(int_marker.pose.position.x, int_marker.pose.position.y);

    // Insert first visual marker
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    control.always_visible = true;

    int_marker.name = "follow_me_" + std::to_string(0) + "_display";
    int_marker.pose.position.x -= 1.0;
    int_marker.pose.position.y -= 1.0;

    int_marker.controls.clear();
    control.markers.clear();
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
    int_srv_->insert(int_marker);

    // Insert follow me visual marker
    marker.color.r = 1.0;
    marker.color.g = 0.0;

    int_marker.name = "follow_me_" + std::to_string(1) + "_display";
    int_marker.pose.position.x += 1.0;
    int_marker.pose.position.y += 1.0;

    int_marker.controls.clear();
    control.markers.clear();
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
    int_srv_->insert(int_marker);
  }

  void IntHandler::displayLoop()
  {
    for (auto &marker : marker_contexts_)
    {
      if (marker.is_active)
      {
        int_srv_->applyChanges();
      }
    }
  }

  void IntHandler::markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int marker_idx)
  {
    marker_contexts_.emplace_back();
  }

  void IntHandler::splitMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const int marker_idx)
  {
    // Update pose
    geometry_msgs::Pose pose = feedback->pose;
    marker_contexts_[marker_idx].x = pose.position.x;
    marker_contexts_[marker_idx].y = pose.position.y;
    int_srv_->setPose("split_marker_" + std::to_string(marker_idx) + "_display", pose);

    if (visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN == feedback->event_type)
    {
      marker_contexts_[marker_idx].is_active = true;
    }
    if (visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP == feedback->event_type)
    {
      marker_contexts_[marker_idx].is_active = false;
    }
  }

  void IntHandler::followMeMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int marker_idx)
  {
    // Update pose
    geometry_msgs::Pose pose = feedback->pose;

    switch (marker_idx)
    {
      case 0:
        marker_contexts_[0].x = pose.position.x;
        marker_contexts_[0].y = pose.position.y;
        int_srv_->setPose("follow_me_" + std::to_string(0) + "_display", pose);
        pose.position.x = marker_contexts_[1].x = pose.position.x + 1.0;
        pose.position.y = marker_contexts_[1].y = pose.position.y + 1.0;
        int_srv_->setPose("follow_me_" + std::to_string(1), pose);
        int_srv_->setPose("follow_me_" + std::to_string(1) + "_display", pose);
        break;

      case 1:
        marker_contexts_[1].x = pose.position.x;
        marker_contexts_[1].y = pose.position.y;
        int_srv_->setPose("follow_me_" + std::to_string(1) + "_display", pose);
        pose.position.x = marker_contexts_[0].x = pose.position.x - 1.0;
        pose.position.y = marker_contexts_[0].y = pose.position.y - 1.0;
        int_srv_->setPose("follow_me_" + std::to_string(0), pose);
        int_srv_->setPose("follow_me_" + std::to_string(0) + "_display", pose);
        break;
    }

    if (visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN == feedback->event_type)
    {
      marker_contexts_[marker_idx].is_active = true;
    }
    if (visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP == feedback->event_type)
    {
      marker_contexts_[marker_idx].is_active = false;
    }
  }
} // interaction
