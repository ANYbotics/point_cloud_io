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
#include <pcl_conversions/pcl_conversions.h>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

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
  if (!nodeHandle_.getParam("file_path", filePath_)) allParametersRead = false;
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
        " _file_path:=path_to_your_point_cloud_file _topic:=/your_topic _frame:=point_cloud_frame"
        " (optional) _rate:=publishing_rate");
    return false;
  }

  return true;
}

void PlyPublisher::initialize()
{
  if (!readFile(filePath_, pointCloudFrameId_)) ros::requestShutdown();

  if (isContinousPublishing_)
  {
    timer_ = nodeHandle_.createTimer(updateDuration_, &PlyPublisher::timerCallback, this);
  }
  else
  {
    Duration(1.0).sleep(); // Need this to get things ready before publishing.
    if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
    ros::requestShutdown();
  }
}

bool PlyPublisher::readFile(const std::string& filePath, const std::string& pointCloudFrameId)
{
  if (filePath.find(".ply") != std::string::npos) {
    // Load .ply file.
    PointCloud<PointXYZ> pointCloud;
    if (loadPLYFile(filePath, pointCloud) != 0) return false;

    // Define PointCloud2 message.
    toROSMsg(pointCloud, *pointCloudMessage_);
  }
  else if (filePath.find(".vtk") != std::string::npos) {
    // Load .vtk file.
    PolygonMesh polygonMesh;
    loadPolygonFileVTK(filePath, polygonMesh);

    // Define PointCloud2 message.
    moveFromPCL(polygonMesh.cloud, *pointCloudMessage_);
  }
  else
  {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  ROS_INFO_STREAM("Loaded point cloud with " << pointCloudMessage_->width << " points.");
  return true;
}

void PlyPublisher::timerCallback(const ros::TimerEvent& timerEvent)
{
  if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
}

bool PlyPublisher::publish()
{
  pointCloudMessage_->header.stamp = Time::now();
  if (pointCloudPublisher_.getNumSubscribers() > 0u)
  {
    pointCloudPublisher_.publish(pointCloudMessage_);
    ROS_INFO_STREAM("Point cloud published in topic \"" << pointCloudTopic_ << "\".");
  }
  return true;
}

} /* namespace */
