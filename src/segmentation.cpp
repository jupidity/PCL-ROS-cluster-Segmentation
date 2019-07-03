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
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <obj_recognition/SegmentedClustersArray.h>
// #include <obj_recognition/ClusterData.h>
#include <pcl/features/normal_3d.h>
#include <std_msgs/Float32.h>



class segmentation {

public:

  explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

    // define the subscriber and publisher
    m_sub = m_nh.subscribe ("/obj_recognition/point_cloud_static", 1, &segmentation::cloud_cb, this);

    // m_sub = m_nh.subscribe ("/camera/depth/points", 1, &segmentation::cloud_cb, this);
    table_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/obj_recognition/table_point_cloud_static", 1);
    table_centroid_pub_x = m_nh.advertise<std_msgs::Float32>("/obj_recognition/table_centroid_x", 1);
    table_centroid_pub_y = m_nh.advertise<std_msgs::Float32>("/obj_recognition/table_centroid_y", 1);
    table_centroid_pub_z = m_nh.advertise<std_msgs::Float32>("/obj_recognition/table_centroid_z", 1);

    m_clusterPub = m_nh.advertise<obj_recognition::SegmentedClustersArray> ("obj_recognition/pcl_clusters",1);

  }
void publish_data();

private:

ros::NodeHandle m_nh;
ros::Publisher m_pub;
ros::Subscriber m_sub;
ros::Publisher m_clusterPub;
ros::Publisher table_pub;
ros::Publisher table_centroid_pub_x;
ros::Publisher table_centroid_pub_y;
ros::Publisher table_centroid_pub_z;
Eigen::Vector4f centroid;
sensor_msgs::PointCloud2 output;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


}; // end class definition


// define callback function
void segmentation::publish_data()
{
  printf("Publishing data");
  std_msgs::Float32 msg;
  msg.data = centroid[0];
  table_centroid_pub_x.publish(msg);

  msg.data = centroid[1];
  table_centroid_pub_y.publish(msg);

  msg.data = centroid[2];
  table_centroid_pub_z.publish(msg);

  // output.header.frame_id = "world";
  table_pub.publish(output);

}
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  int table_segmentation_request;
  if (!ros::param::get("table_segmentation_request", table_segmentation_request))
  {
     ROS_ERROR("Couldnt get parameter table_segmentation_request");
     return;
  }
  if (table_segmentation_request == 1)
  {
      ROS_ERROR("Received request %d", table_segmentation_request);
      ros::param::set("table_segmentation_request", 0);
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


      // Convert to PCL data type
      pcl_conversions::toPCL(*cloud_msg, *cloud);


      // Perform voxel grid downsampling filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.01, 0.01, 0.01);
      sor.filter (*cloudFilteredPtr);


      pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

      // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
      pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


      //perform passthrough filtering to remove table leg

      // create a pcl object to hold the passthrough filtered results
      pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);
      //
      // // Create the filtering object
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (xyzCloudPtr);
      pass.setFilterFieldName ("z");
      // pass.setFilterLimits (.5, 0.7);

      // Use this limits for static point cloud of table to get filtered point cloud without object and ground
      pass.setFilterLimits (.1, 0.7);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*xyzCloudPtrFiltered);



      if (xyzCloudPtrFiltered->size() > 0)
      {
            ROS_ERROR("Was able to filter cloud within given height with non-zero points");

            // create a pcl object to hold the ransac filtered results
            pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


            // perform ransac planar filtration to remove table top
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
            // Optional
            seg1.setOptimizeCoefficients (true);
            // Mandatory
            seg1.setModelType (pcl::SACMODEL_PLANE);
            seg1.setMethodType (pcl::SAC_RANSAC);
            seg1.setDistanceThreshold (0.02);

            seg1.setInputCloud (xyzCloudPtrFiltered);
            // seg1.setInputCloud (xyzCloudPtr);
            seg1.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
              ROS_ERROR ("Could not estimate a planar model for the given dataset.");
              // return (-1);
            }

            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                                << coefficients->values[1] << " "
                                                << coefficients->values[2] << " "
                                                << coefficients->values[3] << std::endl;

            std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
            // for (size_t i = 0; i < inliers->indices.size (); ++i)
            //   std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
            //                                              << cloud->points[inliers->indices[i]].y << " "
            //                                              << cloud->points[inliers->indices[i]].z << std::endl;

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;

            extract.setInputCloud (xyzCloudPtrFiltered);
            // extract.setInputCloud (xyzCloudPtr);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*xyzCloudPtrRansacFiltered);

            pcl::compute3DCentroid(*xyzCloudPtrRansacFiltered,centroid);



            std::cerr << "Plane Centroid: " << centroid[0] << "     " << centroid[1] << "     " << centroid[2] << std::endl;

            // centroid[1] = 1 - centroid[1];

            // std::cerr << "Plane Centroid: " << centroid[0] << "     " << centroid[2] << "     " << centroid[1] << std::endl;
            // std_msgs::Float32 msg;
            // msg.data = centroid[0];
            // table_centroid_pub_x.publish(msg);
            //
            // msg.data = centroid[1];
            // table_centroid_pub_y.publish(msg);
            //
            // msg.data = centroid[2];
            // table_centroid_pub_z.publish(msg);

            // table_centroid_pub_x.publish(msg);

            // msg.data = centroid[1];
            // table_centroid_pub_y.publish(msg);

            // msg.data = centroid[2];
            // table_centroid_pub_z.publish(msg);
            // table_centroid_pub_y.publish(centroid[1]);
            // table_centroid_pub_z.publish(centroid[2]);

            // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            // ne.setInputCloud (xyzCloudPtrRansacFiltered);
            //
            // // Create an empty kdtree representation, and pass it to the normal estimation object.
            // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            // ne.setSearchMethod (tree);
            //
            // // Output datasets
            // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            //
            // // Use all neighbors in a sphere of radius 3cm
            // ne.setRadiusSearch (0.03);
            //
            // // Compute the features
            // ne.compute (*cloud_normals);
            // for (size_t i = 0; i < cloud_normals->points.size (); ++i)
            //   std::cerr  << cloud_normals->points[i].x << " "
            //              << cloud_normals->points[i].y << " "
            //              << cloud_normals->points[i].z << std::endl;
            // // perform euclidean cluster segmentation to seporate individual objects

            // Create the KdTree object for the search method of the extraction
            // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            // tree->setInputCloud (xyzCloudPtrRansacFiltered);
            //
            // // create the extraction object for the clusters
            // std::vector<pcl::PointIndices> cluster_indices;
            // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            // // specify euclidean cluster parameters
            // ec.setClusterTolerance (0.02); // 2cm
            // ec.setMinClusterSize (100);
            // ec.setMaxClusterSize (25000);
            // ec.setSearchMethod (tree);
            // ec.setInputCloud (xyzCloudPtrRansacFiltered);
            // // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
            // ec.extract (cluster_indices);
            //
            // // declare an instance of the SegmentedClustersArray message
            // obj_recognition::SegmentedClustersArray CloudClusters;

            // declare the output variable instances
            // sensor_msgs::PointCloud2 output;
            pcl::PCLPointCloud2 outputPCL;
            pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,outputPCL);

            // Convert to ROS data type
            pcl_conversions::fromPCL(outputPCL, output);

            // add the cluster to the array message
            //clusterData.cluster = output;
            // CloudClusters.clusters.push_back(output);

            // publish the clusters
            // table_pub.publish(output);
            ros::param::set("table_segmentation_done", 1);
      }
  }
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately

  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  // {
  //
  //   // create a new clusterData message object
  //   //obj_recognition::ClusterData clusterData;
  //
  //
  //   // create a pcl object to hold the extracted cluster
  //   pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);
  //
  //   // now we are in a vector of indices pertaining to a single cluster.
  //   // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
  //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  //   {
  //     clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
  //
  //       }
  //
  //
  //   // log the position of the cluster
  //   //clusterData.position[0] = (*cloudPtr).data[0];
  //   //clusterData.position[1] = (*cloudPtr).points.back().y;
  //   //clusterData.position[2] = (*cloudPtr).points.back().z;
  //   //std::string info_string = string(cloudPtr->points.back().x);
  //   //printf(clusterData.position[0]);
  //
  //   // convert to pcl::PCLPointCloud2
  //   pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
  //
  //   // Convert to ROS data type
  //   pcl_conversions::fromPCL(outputPCL, output);
  //
  //   // add the cluster to the array message
  //   //clusterData.cluster = output;
  //   CloudClusters.clusters.push_back(output);
  //
  // }
  //
  // // publish the clusters
  // m_clusterPub.publish(CloudClusters);

}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;
  ros::param::set("table_segmentation_request", 0);
  // ros::param::set("table_segmentation_done", 0);

  segmentation segs(nh);
  ros::Rate r(10); // 10 hz



  while(ros::ok())
  {
      // std::cout << nh.getParam("table_segmentation_request", table_segmentation_done);
      int table_segmentation_done;
      // printf("%d", table_segmentation_done);
      if (ros::param::get("table_segmentation_done", table_segmentation_done))
      {
          if (table_segmentation_done == 1)
          {
              segs.publish_data();
          }
      }
      ros::spinOnce ();
      r.sleep();

  }

}
