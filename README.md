# RViz Interactive Markers

This repository demonstarte the usage of rviz interactive markers. This ROS package rewrites the tutorial in cpp class form. For more information about interactive markers please visit this [page](http://wiki.ros.org/rviz/Tutorials).


## Overview of Interactive Markers

Interactive markers are similar to the "regular" markers described in the previous tutorials, however they allow the user to interact with them by changing their position or rotation, clicking on them or selecting something from a context menu assigned to each marker.  

They are represented by the [visualization_msgs/InteractiveMarker](http://docs.ros.org/api/visualization_msgs/html/msg/InteractiveMarker.html) message, which contains a context menu and several controls ([visualization_msgs/InteractiveMarkerControl](http://docs.ros.org/api/visualization_msgs/html/msg/InteractiveMarkerControl.html)). The controls define the different visual parts of the interactive marker, can consist of several regular markers ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)) and can each have a different function.  

![img](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started?action=AttachFile&do=get&target=interactive_marker_structure.png)  

If you want to create a node providing a set of interactive markers, you need to instantiate an InteractiveMarkerServer object. This will handle the connection to the client (usually RViz) and make sure that all changes you make are being transmitted and that your application is being notified of all the actions the user performs on the interactive markers.  

![img](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started?action=AttachFile&do=get&target=interactive_marker_architecture.png)  


## Interactive Marker Selection

You may have your own combination of interactive marker based on your own preference. Below are some of the combination of interactive markers introduced in the tutorial.  

**Steps to setup interactive markers:**
- Create a interactive marker server (auto int_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer> ("int_server", "", false))
    - Create interactive marker message (visualization_msgs::InteractiveMarker int_marker_msg_)
        - Define int_marker_msg_.header.frame_id (map)
        - Define scale (1)
    - Create a normal marker message (visualization_msgs::Marker marker_msg_)
        - Define marker_msg_.type (visualization_msgs::Marker::CUBE)
        - Define marker_msg_.scale (x, y, z; 0.45, 0.45, 0.45)
        - Define marker_msg_.color (r, g, b, a; 0.5, 0.5, 0.5, 1.0)
    - Create a visualization message (visualization_msgs::InteractiveMarkerControl viz_int_marker_msg_)
        - Define viz_int_marker_msg_.always_visible (true)
    - Create a control message (visualization_msgs::InteractiveMarkerControl con_int_marker_msg_)
- Insert interactive markers into interactive marker server (int_server_->insert(int_marker_msg_, \<lambda function\>)
- Commit and apply changes (int_server_->applyChanges())

**REMEMBER** to add `ros::spin()` or `ros::spinOnce()` in order for the interactive server to work. If not you may encounter look up error.  


Below is an example with free orientation.  

### Free Orientation
```cpp
// Create interactive marker server
auto int_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer> ("int_server", "", false);

// Set interactive marker tf frame
int_marker_msg_.header.frame_id = frame_id_;

// Set interactive marker scale
int_marker_msg_.scale = 1;

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

            /*
            * Insert interactive marker into server
            */

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

            });

            /**
             * 'commit' changes and send to all clients (client = RViz)
             */
            this->int_server_->applyChanges();

            /**
             * Remember to rosspin for callbacks!
             */
```

### Fixed Orientation

### Arbitrary Axes

### Quadcopter

### 3D Motion with mouse

### View Facing

### Chess Piece

### Pan / Tilt

### Context Menu (Right Click)

### Button
