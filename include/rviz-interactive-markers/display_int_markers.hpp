#ifndef __SPM_H_
#define __SPM_H_


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <rviz-interactive-markers/RVizInteractiveMarkerConfig.h>
#include <interactive_markers/interactive_marker_server.h>
// #include <interactive_markers/menu_handler.h>

#include <string>
#include <memory>
#include <random>

namespace interactive_markers_ns
{
    class interactive_marker
    {
        private:
            // ROS declaration
            ros::NodeHandle private_nh_;

            // Interactive marker server
            std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_;
            // Interactive marker menu handler
            // interactive_markers::
            // Dynamic reconfigure handle
            std::shared_ptr<dynamic_reconfigure::Server<rviz_interactive_markers::RVizInteractiveMarkerConfig>> dynamic_server_;

            // Private variable
            visualization_msgs::InteractiveMarker int_marker_msg_;
            visualization_msgs::Marker marker_msg_;
            visualization_msgs::InteractiveMarkerControl viz_int_marker_msg_;
            visualization_msgs::InteractiveMarkerControl con_int_marker_msg_;

            std::string server_name_;
            std::string frame_id_;
            int rate_;
            bool dynamic_callback_;
            int int_marker_mode_;

            // Private function
            void attach_viz();
            void insert_int_marker();
            void remove_int_marker();
            void display_int_marker();

        public:
            // Constructor and destructor
            interactive_marker();
            interactive_marker(std::string);
            ~interactive_marker();

            // Start method
            void start();
    };
};



#endif