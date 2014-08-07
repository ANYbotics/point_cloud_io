ply Publisher
======================

Overview
---------------

This is a simple [ROS] node to read a point cloud from a .ply file and to publish it as a [sensor_msgs/PointCloud2] message. It is meant to be used to visualize reference point clouds in [rviz].

Make sure to set the **Decay Time** in the **PointCloud2** tab in [rviz] to a high number to get the point cloud visible for a long time.

The ply Publisher has been tested under ROS Groovy and Ubuntu 13.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Peter Fankhauser, pfankhauser@ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich**


Installation
------------

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the it depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing).


### Building

In order to install the ply Publisher, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/ply_publisher.git
    cd ../
    catkin_make


Usage
------------

Load and publish a .ply file with

    rosrun ply_publisher ply_publisher _file_path:=/home/user/your_point_cloud.ply _topic:=/point_cloud _frame:=/sensor_frame

Optionally, you can also add `_rate:=1.0` to have the node publish your point cloud at the specified rate. To create your own launch-file, you can use the example from `ply_publisher/launch/example.launch`.


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ply_publisher/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html

