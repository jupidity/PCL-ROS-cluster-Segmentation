/*******************************************************************************
 * Copyright (C) 2017 Electric Movement Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Brandon Kinman

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <sensor_msgs/PointCloud2.h>

#include <obj_recognition/GetNormals.h>
//#include <obj_recognition/GetFloatArrayFeature.h>

/*
* Brief:
* This node generates normal features for a point cloud
*/

class FeatureExtractor
{
public:
  explicit FeatureExtractor(ros::NodeHandle nh)
    : nh_(nh)
  {
    // Define Publishers and Subscribers here
    cluster_in_sub_ = nh_.subscribe("cluster_in", 1, &FeatureExtractor::clusterCallback, this);
    normals_out_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normals_out", 1);
    get_normals_srv_ = nh_.advertiseService("get_normals", &FeatureExtractor::getNormalsReq, this);
    //get_vfh_srv_ = np_.advertiseService("get_vfh", &FeatureExtractor::getVFHReq, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cluster_in_sub_;
  ros::Publisher normals_out_pub_;
  ros::ServiceServer get_normals_srv_;

  void clusterCallback(const sensor_msgs::PointCloud2& cloud_msg)
  {
    ROS_INFO("Cluster Received");

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(cloud_msg, *p_cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Compute the features
    ne.compute(*cloud_normals);

    ROS_INFO("Done!");

    sensor_msgs::PointCloud2 normals_out_msg;
    pcl::toROSMsg(*cloud_normals, normals_out_msg);

    normals_out_pub_.publish(normals_out_msg);
  }

  bool getNormalsReq(obj_recognition::GetNormals::Request &req, obj_recognition::GetNormals::Response &rsp)
  {
    rsp.cluster = req.cluster;

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(req.cluster, *p_cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Compute the features
    ne.compute(*cloud_normals);

    pcl::toROSMsg(*cloud_normals, rsp.cluster);

    return true;
  }

};  // FeatureExtractor

// bool getVFHReq(obj_recognition::GetNormals::Request &req, obj_recognition::GetNormals::Response &rsp)
// {
//   pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
//   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
//   pcl::fromROSMsg(req.cluster, *p_cloud);

//   // Create the normal estimation class, and pass the input dataset to it
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   ne.setInputCloud (sp_pcl_cloud);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
//   ne.setSearchMethod (tree);

//   // Use all neighbors in a sphere of radius 3cm
//   ne.setRadiusSearch(0.03);

//   // Output datasets
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

//   // Compute the features
//   ne.compute(*cloud_normals);

//   // Create the VFH estimation class, and pass the input dataset+normals to it
//   pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
//   vfh.setInputCloud(sp_pcl_cloud);
//   vfh.setInputNormals(*cloud_normals);
//   // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

//   // Create an empty kdtree representation, and pass it to the FPFH estimation object.
//   // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//   vfh.setSearchMethod (tree);

//   // Output datasets
//   pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

//   // Compute the features
//   vfh.compute (*vfhs);

//   for(size_t i = 0; i < 308; ++i)
//   {
//     vfhs.points[0].histogram[i];
//   }


// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_extractor");
  ros::NodeHandle nh("~");

  FeatureExtractor nfe(nh);

  // Spin until ROS is shutdown
  while (ros::ok())
    ros::spin();

  return 0;
}
