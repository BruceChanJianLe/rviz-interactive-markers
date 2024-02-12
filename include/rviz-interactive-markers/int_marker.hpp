/**
 * Another example of interactive marker
 * Splitting int marker and display marker
 */
#ifndef _RVIZ_INTERACTIVE_MARKERS_INT_MARKER_HPP_
#define _RVIZ_INTERACTIVE_MARKERS_INT_MARKER_HPP_

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

// STL
#include <memory>
#include <vector>

static constexpr float UPDATE_RATE = 1.0f / 20.0f;

namespace interaction
{
  class IntHandler
  {
  public:
    IntHandler();
    ~IntHandler();

    void run();
  private:
    // Node Handlers
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer display_loop_;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_srv_;

    std::string frame_id_;
    int marker_style_;

    int marker_idx_;
    struct MarkerContext
    {
      MarkerContext() : x{0}, y{0}, is_active{false} {}
      MarkerContext(const float x, const float y) : x{x}, y{y}, is_active{false} {}
      float x;
      float y;
      bool is_active;
    };
    std::vector<MarkerContext> marker_contexts_;

    // Private methods
    void addNodeMarker();
    void addSplitNodeMarker();
    void addFollowMeNodeMarker();
    void displayLoop();
    void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int marker_idx);
    void splitMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const int marker_idx);
    void followMeMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const int marker_idx);
  };

} // interaction

#endif /* _RVIZ_INTERACTIVE_MARKERS_INT_MARKER_HPP_ */
