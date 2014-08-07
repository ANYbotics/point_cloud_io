/*
 * PlyPublisher.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ply_publisher/PlyPublisher.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/ros/conversions.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;

namespace ply_publisher {

PlyPublisher::PlyPublisher(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloudMessage_(new sensor_msgs::PointCloud2)
{
  if (!readParameters()) ros::requestShutdown();
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
  initialize();
}

PlyPublisher::~PlyPublisher()
{

}

bool PlyPublisher::readParameters()
{
  bool allParametersRead = true;
  if (!nodeHandle_.getParam("file_path", plyFilePath_)) allParametersRead = false;
  if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;
  if (!nodeHandle_.getParam("frame", pointCloudFrameId_)) allParametersRead = false;

  double updateRate;
  nodeHandle_.param("rate", updateRate, 0.0);
  if (updateRate == 0.0)
  {
    isContinousPublishing_ = false;
  }
  else
  {
    isContinousPublishing_ = true;
    updateDuration_.fromSec(1.0 / updateRate);
  }

  if (!allParametersRead)
  {
    ROS_WARN("Could not read all parameters. Typical command-line usage:\n rosrun ply_publisher ply_publisher"
        " _file_path:=path_to_your_ply_file _topic:=your_topic _frame:=/point_cloud_frame");
    return false;
  }

  return true;
}

void PlyPublisher::initialize()
{
  readFile(plyFilePath_, pointCloudFrameId_);

  if (isContinousPublishing_)
  {
    timer_ = nodeHandle_.createTimer(updateDuration_, &PlyPublisher::timerCallback, this);
  }
  else
  {
    Duration(1.0).sleep(); // Need this to get things ready before publishing.
    if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the ply file.");
    ros::requestShutdown();
  }
}

bool PlyPublisher::readFile(const std::string& filePath, const std::string& pointCloudFrameId)
{
  // Load .ply file.
  PointCloud<pcl::PointXYZ> pointCloud;
  if (loadPLYFile(filePath, pointCloud) != 0) return false;

  // Define PointCloud2 message.
  toROSMsg(pointCloud, *pointCloudMessage_);
  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  ROS_INFO_STREAM("Loaded point cloud with " << pointCloud.size() << " points.");
  return true;
}

void PlyPublisher::timerCallback(const ros::TimerEvent& timerEvent)
{
  if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the ply file.");
}

bool PlyPublisher::publish()
{
  pointCloudMessage_->header.stamp = Time::now();
  pointCloudPublisher_.publish(pointCloudMessage_);
  ROS_INFO_STREAM("Point cloud published in topic \"" << pointCloudTopic_ << "\".");
  return true;
}

} /* namespace */
