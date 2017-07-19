/*

ROS node for point cloud cluster based segmentaion of cluttered objects on table

Author: Sean Cassero
7/15/15

*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloudFilteredPtr);


  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // create a pcl object to hold the filtered results
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZ>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


  pcl::PCLPointCloud2 outputPCL;
  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzCloudPtr ,outputPCL);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(outputPCL, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/sensor_stick/point_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_objects", 1);

  // Spin
  ros::spin ();
}
