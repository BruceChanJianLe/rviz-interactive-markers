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


### Free Orientation

### Fixed Orientation

### Arbitrary Axes

### Quadcopter

### 3D Motion with mouse

### View Facing

### Chess Piece

### Pan / Tilt

### Context Menu (Right Click)

### Button