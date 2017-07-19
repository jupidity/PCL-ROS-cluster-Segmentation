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
#include <pcl/conversions.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>




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
  void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    // ROS messages pass point clouds in different formats then pcl is used to handling
    // in order to use PCL we need to change the data type to a PCL standard type

    // create a container to hold the point cloud after conversion to pcl::PCLPointCloud2
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // call the const ptr constructor and pass it the newly created ptr 'cloud'
    // create a container to hold the point cloud after filtration

    // this dynamically allocates memory for a pcl::PCLPointCloud2 object
    // since new returns a pointer to the created class, we pass the reference to a pcl::PCLPointCloud2 pointer
    pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
    // the pcl lib works with the boost::shared_ptr type for function calls, we need to cast the newly created
    // pointer as such before using it further. PCL typedefs the ConstPtr and Ptr as shared_ptrs, we just need to
    // pass the pointer to a constructor.
    pcl::PCLPointCloud2Ptr cloudPtrFiltered(cloud_filtered) ;

    // Convert to PCL data type. Since the could_msg and cloud vars are pointers, we need to dereference them
    // before passing to the pcl_conversions function
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Create the passthrough filtering object
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr); // since we are working with the pcl::PCLPointCloud2 template, we need to pass a PCLPointCloud2ConstPtr to this function
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloudPtrFiltered);


    // Perform voxel downsample filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor; // declare the voxel grid object of type PCLPointCloud2
    sor.setInputCloud(cloudPtrFiltered);
    sor.setLeafSize(0.1, 0.1, 0.1); // specify the leaf size
    sor.filter(*cloudPtrFiltered); // perform the filtration


    // perform RANSAC filtration

    // new dynamically allocates memory and returns a pointer to the new variable on the heap. here we are creating
    // a new point cloud data structure to pass to the ransac function
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

    // create a pcl object to hold the filtered results
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

    // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromPCLPointCloud2(*cloudPtrFiltered, *xyzCloudPtr);

    // now we wish to create a new ransac plane model
    //pcl::SampleConsensusModelPlane<pcl::PointXYZ> *ransac_plane_model = new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (xyzCloudPtr);
    // cast as a shared_ptr
    //pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr planeModelPtr(ransac_plane_model);

    // perform ransac segmentation

    // std::vector<int> inliers;
    // get a pointer to indices
    //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // create the ransac object and pass it the plane model
    //pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(planeModelPtr);

    // specify ransac parameters and compute
    //ransac.setDistanceThreshold (.01);
    //ransac.computeModel();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (xyzCloudPtr);
    seg.segment (*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (xyzCloudPtr);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*xyzCloudPtrFiltered);


    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(*xyzCloudPtrFiltered,*cloudPtrFiltered);
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloudPtrFiltered, output);

    // Publish the data
    m_pub.publish(output);

  }

};





int main(int argc, char **argv){

  // initialize the node
  ros::init(argc, argv, "segmentation");

  // start a class of the publisher and subscriber
  pclPubSub segmentation;

  // while node is not shutdown, wait for messages
  while(ros::ok()){
    ros::spin();
  }

}
