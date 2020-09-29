# RViz Interactive Markers

This repository demonstarte the usage of rviz interactive markers. This ROS package rewrites the tutorial in cpp class form. For more information about interactive markers please visit this [page](http://wiki.ros.org/rviz/Tutorials).


## Overview of Interactive Markers

Interactive markers are similar to the "regular" markers described in the previous tutorials, however they allow the user to interact with them by changing their position or rotation, clicking on them or selecting something from a context menu assigned to each marker.  

They are represented by the [visualization_msgs/InteractiveMarker](http://docs.ros.org/api/visualization_msgs/html/msg/InteractiveMarker.html) message, which contains a context menu and several controls ([visualization_msgs/InteractiveMarkerControl](http://docs.ros.org/api/visualization_msgs/html/msg/InteractiveMarkerControl.html)). The controls define the different visual parts of the interactive marker, can consist of several regular markers ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)) and can each have a different function.  

![img](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started?action=AttachFile&do=get&target=interactive_marker_structure.png)  

If you want to create a node providing a set of interactive markers, you need to instantiate an InteractiveMarkerServer object. This will handle the connection to the client (usually RViz) and make sure that all changes you make are being transmitted and that your application is being notified of all the actions the user performs on the interactive markers.  

![img](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started?action=AttachFile&do=get&target=interactive_marker_architecture.png)  
