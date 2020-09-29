#ifndef __SM_H_
#define __SM_H_


#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <string>


class simple_marker
{
    private:
        // ROS declaration
        ros::NodeHandle private_nh_;
        // Interactive marker msg
        visualization_msgs::InteractiveMarker int_marker_msg_;
        // Marker msg
        visualization_msgs::Marker marker_msg_;
        // Visualization for interactive marker msg
        visualization_msgs::InteractiveMarkerControl viz_int_marker_con_msg_;
        // Control for interactive marker msg
        visualization_msgs::InteractiveMarkerControl con_int_marker_msg_;

        // Private variables
        int rate_;
        std::string frame_id_;
        std::string server_name_;

    public:
        // Constructor and destructor
        simple_marker();
        simple_marker(std::string);
        ~simple_marker();

        // Public function
        void start();
};


#endif