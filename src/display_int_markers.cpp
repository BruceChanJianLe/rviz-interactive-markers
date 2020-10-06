#include "rviz-interactive-markers/display_int_markers.hpp"


namespace interactive_markers_ns
{
    interactive_marker::interactive_marker()
    {
        ;
    }


    interactive_marker::~interactive_marker()
    {
        ;
    }


    interactive_marker::interactive_marker(std::string server_name)
    :   int_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>(server_name, "", false)),
        dynamic_server_(std::make_shared<dynamic_reconfigure::Server<rviz_interactive_markers::RVizInteractiveMarkerConfig>> ())
    {
        // Set callback function for dynamic reconfigure (using lambda)
        dynamic_server_->setCallback(
            [this](rviz_interactive_markers::RVizInteractiveMarkerConfig & config, uint32_t level)
            {
                this->dynamic_callback_ = true;
                this->int_marker_mode_ = config.marker_drop_down_selection;
                // ROS_INFO_STREAM("The current marker is: " << this->int_marker_mode_);
            }
        );

        // Load parameters
        private_nh_.param("rate", rate_, 5);
        private_nh_.param<std::string>("frame_id", frame_id_, "map");
    }


    void interactive_marker::insert_int_marker()
    {

        // Method 1
            int_server_->insert(int_marker_msg_,
            [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
            {
                // Create a string to hold message
                std::string message;

                // Obtain marker name and control name
                message = "\nMarker Name: " + feedback->marker_name +
                          "\nControl Method: " + feedback->control_name;

                // Obtain mouse point (if exist)
                if(feedback->mouse_point_valid)
                {
                    message += "\nMouse Position: (" + std::to_string(feedback->mouse_point.x) +
                               ", " + std::to_string(feedback->mouse_point.y) +
                               ", " + std::to_string(feedback->mouse_point.z) +
                               ")" +
                               "\nFrame: " + feedback->header.frame_id;
                }

                // Determine feedback event
                switch (feedback->event_type)
                {
                case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
                    ROS_INFO_STREAM(message << "\nEvent Type: button click");
                    break;

                case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                    ROS_INFO_STREAM(
                        message << "\nEvent Type: menu item" <<
                        "\nMenu Entry: " << feedback->menu_entry_id
                    );
                    break;

                case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
                    ROS_INFO_STREAM(
                        message << "\nEvent Type: pose update" <<
                        "\nMarker Position: (" << feedback->pose.position.x <<
                        ", " << feedback->pose.position.y << 
                        ", " << feedback->pose.position.z <<
                        ")" <<
                        "\nMarker Orientation: (" << feedback->pose.orientation.x <<
                        ", " << feedback->pose.orientation.y <<
                        ", " << feedback->pose.orientation.z <<
                        ", " << feedback->pose.orientation.w <<
                        ")"
                    );
                    break;

                case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
                    ROS_INFO_STREAM(
                        message << "\nEvent Type: mouse down"
                    );
                    break;

                case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
                    ROS_INFO_STREAM(
                        message << "\nEvent Type: mouse up"
                    );
                    break;

                default:
                    break;
                }

                // 'commit' changes and send to all clients (client = RViz)
                this->int_server_->applyChanges();
            });

        // Method 2
            // Insert the defined interactive marker to the server (you may set the callback in this method)
            // int_server_->insert(int_marker_msg_);

            // Setcallback function for the respective marker
            // int_server_->setCallback(int_marker_msg_.name,
            // []()
            // {
            //     ;
            // });

    }


    void interactive_marker::remove_int_marker()
    {
        // Remove any inserted interactive markers
        int_server_->clear();

    }


    void interactive_marker::prepare_int_marker()
    {
        // Set interactive marker tf frame
        int_marker_msg_.header.frame_id = frame_id_;

        // Set interactive marker scale
        int_marker_msg_.scale = 1;

        // int_marker_msg_.pose.position.x = 0.0;
        // int_marker_msg_.pose.position.y = 0.0;
        // int_marker_msg_.pose.position.z = 0.0;
        // int_marker_msg_.pose.orientation.x = 0.0;
        // int_marker_msg_.pose.orientation.y = 0.0;
        // int_marker_msg_.pose.orientation.z = 0.0;
        // int_marker_msg_.pose.orientation.w = 0.0;

        // Clear interactive marker controller's vector
        int_marker_msg_.controls.clear();

        // Clear controller settings for con_int_marker_msg_
        con_int_marker_msg_ = std::move(visualization_msgs::InteractiveMarkerControl ());

        // Prepare visual marker for interactive marker

            // Marker visual
            marker_msg_.type = visualization_msgs::Marker::CUBE;

            // Scale marker (obtain it from the interactive marker)
            marker_msg_.scale.x = int_marker_msg_.scale * 0.45;
            marker_msg_.scale.y = int_marker_msg_.scale * 0.45;
            marker_msg_.scale.z = int_marker_msg_.scale * 0.45;

            // Color marker
            marker_msg_.color.r = 0.5;
            marker_msg_.color.g = 0.5;
            marker_msg_.color.b = 0.5;
            marker_msg_.color.a = 1.0;

            // Prepare visualization for interactive marker
            viz_int_marker_msg_.always_visible = true;
            viz_int_marker_msg_.markers.emplace_back(marker_msg_);

        switch(int_marker_mode_)
        {
            case 0:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Free 6D Control";
                int_marker_msg_.description = "Free Orientation";

                // Set visualization for marker to be free control
                viz_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

                // Display controls
                    // Rotate x axis
                    con_int_marker_msg_.name = "rotate_x_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 1;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate x axis
                    con_int_marker_msg_.name = "translate_x_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Rotate y axis
                    con_int_marker_msg_.name = "rotate_y_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 1;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate y axis
                    con_int_marker_msg_.name = "translate_y_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Rotate z axis
                    con_int_marker_msg_.name = "rotate_z_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 1;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate y axis
                    con_int_marker_msg_.name = "translate_z_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Attach visual marker to interactive marker
                    int_marker_msg_.controls.emplace_back(viz_int_marker_msg_);
                break;
            }

            case 1:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Fixed 6D Control";
                int_marker_msg_.description = "Fixed Orientation";

                // Set visualization for marker to be free control
                viz_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

                // Display controls
                    // Rotate x axis
                    con_int_marker_msg_.name = "rotate_x_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 1;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate x axis
                    con_int_marker_msg_.name = "translate_x_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Rotate y axis
                    con_int_marker_msg_.name = "rotate_y_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 1;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate y axis
                    con_int_marker_msg_.name = "translate_y_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Rotate z axis
                    con_int_marker_msg_.name = "rotate_z_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 1;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate y axis
                    con_int_marker_msg_.name = "translate_z_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Attach visual marker to interactive marker
                    int_marker_msg_.controls.emplace_back(viz_int_marker_msg_);
                break;
            }

            case 2:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Random 6D Control";
                int_marker_msg_.description = "Arbitary Axes";

                // Display control
                    // Use random engine to generate value between -1.0 to 1.0
                    std::random_device rd;
                    std::mt19937 eng(rd());
                    std::uniform_real_distribution<> distr_double(-1.0, 1.0);

                    // Provide three random axis for rotate and translate
                    for(int i = 0; i < 3; i++)
                    {
                        // Random rotate axis
                        con_int_marker_msg_.name = "random_rotate_axis_" + std::to_string(i);
                        con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                        con_int_marker_msg_.orientation.x = distr_double(eng);
                        con_int_marker_msg_.orientation.y = distr_double(eng);
                        con_int_marker_msg_.orientation.z = distr_double(eng);
                        con_int_marker_msg_.orientation.w = distr_double(eng);
                            // Attach random rotate axis
                            int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                        // Random translate axis
                        con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                            // Attach random translate axis
                            int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    }

                    // Attach visual marker to interactive marker
                    int_marker_msg_.controls.emplace_back(viz_int_marker_msg_);
                break;
            }

            case 3:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Quad Cop 6D Control";
                int_marker_msg_.description = "Quadcopter";

                // Display control
                    // Quadcopter move rotate axis
                    con_int_marker_msg_.name = "move_rotate_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
                    con_int_marker_msg_.orientation.x = 0.0;
                    con_int_marker_msg_.orientation.y = 1.0;
                    con_int_marker_msg_.orientation.z = 0.0;
                    con_int_marker_msg_.orientation.w = 1.0;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Quadcopter translate axis
                    con_int_marker_msg_.name = "translate_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Attach visual marker to interactive marker
                    int_marker_msg_.controls.emplace_back(viz_int_marker_msg_);
                break;
            }

            case 4:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Fixed 6D 3D Control";
                int_marker_msg_.description = "3D Motion with mouse";

                // Set visualization for marker to be free control
                viz_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

                // Display control
                    // Rotate x axis
                    con_int_marker_msg_.name = "rotate_x_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 1;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate x axis
                    con_int_marker_msg_.name = "translate_x_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Rotate y axis
                    con_int_marker_msg_.name = "rotate_y_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 1;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate y axis
                    con_int_marker_msg_.name = "translate_y_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Rotate z axis
                    con_int_marker_msg_.name = "rotate_z_axiz";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 1;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Translate y axis
                    con_int_marker_msg_.name = "translate_z_axis";
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Attach visual marker to interactive marker
                    int_marker_msg_.controls.emplace_back(viz_int_marker_msg_);
                break;
            }

            case 5:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "View Facing 6D 3D Control";
                int_marker_msg_.description = "View-Facing";

                // Display control
                    // Control of rotates around the viewing axis
                    con_int_marker_msg_.name = "rotate_viewing_face_control";
                    con_int_marker_msg_.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation.w = 1;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Control of translation around the viewing axis
                    con_int_marker_msg_.name = "translate_viewing_face_control";
                    con_int_marker_msg_.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
                    con_int_marker_msg_.independent_marker_orientation = true;
                    con_int_marker_msg_.always_visible = true;
                        // Attach visual marker to control marker (something different)
                        con_int_marker_msg_.markers.emplace_back(viz_int_marker_msg_);
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                break;
            }

            case 6:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Planar 2D Control";
                int_marker_msg_.description = "Chess Piece";

                // Display control
                    // Planar movement control
                    con_int_marker_msg_.name = "planar_control";
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 1;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                    con_int_marker_msg_.independent_marker_orientation = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

                    // Prepare visualization marker to attach to control marker
                    con_int_marker_msg_.markers.emplace_back(viz_int_marker_msg_);
                    con_int_marker_msg_.always_visible = true;
                        // Attach control marker with visualize marker in it to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                break;
            }

            case 7:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Pan Tilt";
                int_marker_msg_.description = "Pan / Tilt";

                // Display control
                    // Pan control ??
                    con_int_marker_msg_.name = "pan?";
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 1;
                    con_int_marker_msg_.orientation.z = 0;
                    con_int_marker_msg_.orientation.w = 1;
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                    // Tilt control ??
                    con_int_marker_msg_.name = "tilt?";
                    con_int_marker_msg_.orientation.x = 0;
                    con_int_marker_msg_.orientation.y = 0;
                    con_int_marker_msg_.orientation.z = 1;
                    con_int_marker_msg_.orientation.w = 1;
                    con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                    con_int_marker_msg_.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
                        // Attach control to interactive marker
                        int_marker_msg_.controls.emplace_back(con_int_marker_msg_);
                break;
            }

            case 8:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Context Menu";
                int_marker_msg_.description = "Context Menu";
                break;
            }

            case 9:
            {
                // Define interactive marker name and description
                int_marker_msg_.name = "Button";
                int_marker_msg_.description = "Button";
                break;
            }

        }

        // Inseart interactive marker into the interactive marker server
        insert_int_marker();

    }


    void interactive_marker::start()
    {
        // Set process rate
        ros::Rate r(rate_);

        // Run
        while(private_nh_.ok())
        {
            // Check if interactive marker is updated
            if(dynamic_callback_)
            {
                remove_int_marker();
                prepare_int_marker();
            }

            // ROS spin
            ros::spinOnce();

            // Rest
            r.sleep();
        }


        // // Testing definition
        // visualization_msgs::InteractiveMarker int_marker_msg_;
        // visualization_msgs::Marker marker_msg_;
        // visualization_msgs::InteractiveMarkerControl viz_int_marker_msg_;
        // visualization_msgs::InteractiveMarkerControl con_int_marker_msg_;

        // // Define interactive int_marker_msg_
        // int_marker_msg_.header.frame_id = frame_id_;
        // int_marker_msg_.header.stamp = ros::Time::now();
        // int_marker_msg_.name = "simple_marker";
        // int_marker_msg_.description = "Simple 1-DOF Control";

        // // Define marker_msg_
        // marker_msg_.type = visualization_msgs::Marker::CUBE;
        // marker_msg_.scale.x = 0.45;
        // marker_msg_.scale.y = 0.45;
        // marker_msg_.scale.z = 0.45;
        // marker_msg_.color.r = 0.5;
        // marker_msg_.color.g = 0.5;
        // marker_msg_.color.b = 0.5;
        // marker_msg_.color.a = 1.0;

        // // Define visualization for interactive marker
        // viz_int_marker_msg_.always_visible = true;
        // viz_int_marker_msg_.markers.emplace_back(marker_msg_);

        // // Add visualization to interactive marker msg
        // int_marker_msg_.controls.emplace_back(viz_int_marker_msg_);

        // // Define control for interactive marker
        // con_int_marker_msg_.name = "move_x";
        // con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // // Add control to interactive marker msg
        // int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

        // // Insert interactive marker msg t server
        // int_server_->insert(int_marker_msg_,
        // [](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
        // {
        //     ROS_INFO_STREAM("\n" << feedback->marker_name << 
        //                     " is now at " << feedback->pose.position.x <<
        //                     ", " << feedback->pose.position.y <<
        //                     ", " << feedback->pose.position.z
        //                     );
        // });

        // // 'commit' changes and send to all clients
        // int_server_->applyChanges();

        // while (private_nh_.ok())
        // {
        //     // Allow callbacks
        //     ros::spinOnce();
        //     // Let node sleep
        //     r.sleep();
        // }
    }
};