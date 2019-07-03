/*******************************************************************************
 * Copyright (C) 2017 Electric Movement Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

/*
* Brief:
* This node transforms point cloud from /camera_link frame to /world frame
*/

class CloudTransformer
{
public:
  explicit CloudTransformer(ros::NodeHandle nh)
    : nh_(nh)
  {
    // Define Publishers and Subscribers here
    pcl_sub_ = nh_.subscribe("/head_camera/depth/points", 1, &CloudTransformer::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obj_recognition/point_cloud", 1);

    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = "world";
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener listener_;
  sensor_msgs::PointCloud2::Ptr buffer_;
  tf::StampedTransform transform;
  int cloud_transformer_request;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
  {
    // listener_.waitForTransform("imu", "camera_link", ros::Time::now(), ros::Duration(1000.0));
    // if (nh_.getParam("cloud_transformer_request", cloud_transformer_request) == 1) {
        // nh_.setParam("cloud_transformer_request", 0);
        listener_.lookupTransform("world", "head_camera_depth_optical_frame", ros::Time(0), transform);
        pcl_ros::transformPointCloud("world", transform, *pcl_msg, *buffer_);
    // }
    // if (buffer_->data.size() > 0)
    // {
        // nh_.setParam("cloud_transformer_done", 1);
        pcl_pub_.publish(buffer_);
    // }
  }
};  // End of class CloudTransformer

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_tf");
  ros::NodeHandle nh;

  CloudTransformer tranform_cloud(nh);
  // nh.setParam("cloud_transformer_request", 0);
  // Spin until ROS is shutdown
  while (ros::ok())
    ros::spin();

  return 0;
}
