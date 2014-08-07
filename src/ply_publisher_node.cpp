/*
 * ply_publisher_node.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "ply_publisher/PlyPublisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ply_publisher");
  ros::NodeHandle nodeHandle("~");

  ply_publisher::PlyPublisher plyPublisher(nodeHandle);

  ros::spin();
  return 0;
}
