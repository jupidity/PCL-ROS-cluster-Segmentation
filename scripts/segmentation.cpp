/*

ROS node for point cloud cluster based segmentaion of cluttered objects on table

Author: Sean Cassero
7/15/15

*/



#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



class pclPubSub{


private:
  // declare the node handle and the publishers and subscribers
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;



public:

  pclPubSub(){
    // link the publishers and subscribers to their topics and message types

    // the node in this example gets the PointCloud2 message from the /sensor_stick/point_cloud topic
    ros::Subscriber m_sub = m_nh.subscribe<sensor_msgs::PointCloud2> ("/sensor_stick/point_cloud", 1, &pclPubSub::callback, this);
    // we wish to publish a ros PointCloud2 to the /pcl_objects topic
    ros::Publisher m_pub = m_nh.advertise<sensor_msgs::PointCloud2>("pcl_objects", 1);

  }

  // define the callback function when the node recieves a message from the /sensor_stick/point_cloud topic
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    m_pub.publish(output);

  }

};





int main(int argc, char **argv){

  // initialize the node
  ros::init(argc, argv, "segmentation");

  // create the publishers and subscribers
  // the node in this example gets the PointCloud2 message from the /sensor_stick/point_cloud topic
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/sensor_stick/point_cloud", 1, callback);
  // we wish to publish a ros PointCloud2 to the /pcl_objects topic
  //ros::Publisher pcl_objects_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_objects", 1);



  while(ros::ok()){
    ros::spin();
  }

}
