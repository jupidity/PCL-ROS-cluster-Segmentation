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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>



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

  // we pass a const pointer to the received message of the PointCloud2 type to the callback function
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {

    // ROS messages pass point clouds in different formats then pcl is used to handling
    // in order to use PCL we need to change the data type to a PCL standard type

    // create a container to hold the point cloud after conversion to pcl::PCLPointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // call the const ptr constructor and pass it the newly created ptr 'cloud'
    // create a container to hold the point cloud after filtration
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudPtrFiltered(cloud_filtered) ;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Create the passthrough filtering object
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);


    // Perform voxel downsample filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor; // declare the voxel grid object of type PCLPointCloud2
    sor.setInputCloud(cloudPtrFiltered);
    sor.setLeafSize(0.1, 0.1, 0.1); // specify the leaf size
    sor.filter(*cloud_filtered); // perform the filtration


    // perform RANSAC filtration
    pcl::PointCloud<pcl::PCLPointCloud2>::Ptr ransac_input_cloud (new pcl::PointCloud<pcl::PCLPointCloud2>);


    // the ransac filtration uses the pcl::PointCloud<pcl::PCLPointCloud2> template for calculations
    // we are currently working in the pcl::PCLPointCloud data type. We must do conversions before calcs

    //pcl::PointCloud<pcl::PCLPointCloud2> ransac_input_cloud;

    //pcl::fromPCLPointCloud2(*cloud_filtered, ransac_input_cloud);
    std::vector<int> inliers; // create a vector of ints to hold the indices of the inliers
    pcl::SampleConsensusModelPlane<pcl::PCLPointCloud2>::Ptr ransac_plane(new pcl::SampleConsensusModelPlane<pcl::PCLPointCloud2> (ransac_input_cloud));
    pcl::RandomSampleConsensus<pcl::PCLPointCloud2> ransac (ransac_plane);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);





    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);

    // Publish the data
    m_pub.publish(output);

  }

};





int main(int argc, char **argv){

  // initialize the node
  ros::init(argc, argv, "segmentation");

  // while node is not shutdown, wait for messages
  while(ros::ok()){
    ros::spin();
  }

}
