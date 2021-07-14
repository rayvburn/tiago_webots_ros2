# tiago_webots_ros2

Metapackage with TIAGo robot definitions for Webots simulator with ROS2 interface

## Description

Main contribution of this repository is **ready-to-use** `TIAGo Iron` robot driver for Webots with ROS2 interface.

Package is based on `turtlebot3` metapackage for ROS2 with a great effort of [`renan028`](https://github.com/renan028/tiago_webots_ros2) to tune the navigation parameters.

What's not working (in Webots-ROS2 interface, I suppose):

* `TouchSensor`
* not all joints' Transform Frames (tf2_ros) are published, therefore some of the transformations are hard-coded based on `.proto` file translation and rotation values

## Launch

Default, `intralogistics` world:

```bash
ros2 launch tiago_webots_ros2_driver tiago_webots.launch.py
ros2 launch tiago_webots_ros2_navigation tiago_navigation.launch.py use_sim_time:=True
```

Example world known from TurtleBot3 simulation:

```bash
ros2 launch tiago_webots_ros2_driver tiago_webots.launch.py world_file:=turtlebot3_burger_example.wbt
ros2 launch tiago_webots_ros2_navigation tiago_navigation.launch.py use_sim_time:=True map_file:=$(ros2 pkg prefix tiago_webots_ros2_driver --share)/resource/map/turtlebot3_burger_example.yaml
```

## TODO

TODO: prep rviz for driver pkg

TODO: how to get a perfect map of the world?

One may notice that a custom `.proto` for Hokuyo Lidar was defined (based on Cyberbotics' `HokuyoUrg04lxug01.proto`). This is due to the fact that default [Hokuyo URG-04LX-UG01](https://github.com/cyberbotics/webots/blob/master/projects/devices/hokuyo/protos/HokuyoUrg04lxug01.proto) has a wider `fieldOfView` than the mobile base construction allows. Therefore, edges of the Lidar niche were marked as obstacles in a costmap. See image for reference:



