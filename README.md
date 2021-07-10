# tiago_webots_ros2

Meta

## Description

Package is based on `turtlebot3` metapackage for ROS2 with a great effort of [`renan028`](https://github.com/renan028/tiago_webots_ros2) to tune the navigation parameters.

What's not working (in Webots-ROS2 interface, I suppose):

* `TouchSensor`
* not all joints' Transform Frames (tf2_ros) are published, therefore some of the transformations are hard-coded based on `.proto` file translation and rotation values


TODO: initial pose
TODO: prep rviz for driver pkg