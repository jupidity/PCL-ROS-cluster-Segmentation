[image1]: ./photos/originalPointCloud.png
[image2]: ./photos/VoxelPointCloud.png
[image3]: ./photos/passthroughPointCloud.png
[image4]: ./photos/ransacPointCloud.png
[image5]: ./photos/passthroughEdgePointCloud.png
[image6]: ./photos/finalSegmentation.png
[image7]: ./photos/nodeDuration.png

### ROS Node for Cluster Based Segmentation with PCL
---

`C++` `ROS` node for image segmentation on a cluttered table with cluster based methods. Tested in simulation environment provided at https://github.com/udacity/RoboND-Perception-Exercises.git. Node sensor input is `ROS` msg `PointCloud2` generated in `Gazebo` simulation environment from RGBD camera.`ROS` node `segmentation.cpp` in ``/scripts`` directory.   

### Installation
---

clone the repo:

    git clone https://github.com/jupidity/PCL-ROS-cluster-Segmentation.git

run catkin_make in your ROS source directory

    $ cd ~/catkin_ws
    $ catkin_make

start the simulation with roslaunch

    $ roslaunch roslaunch sensor_stick robot_spawn.launch

start the segmentation node

    $ rosrun sensor_stick segmentation

the segmentation node publishes `sensor_msgs::PCLPointCloud2` messages to the `/pcl_objects` topic. You can visualize the segmentation in `RViz` by selecting that topic to view.


### Dependencies
---

pcl 1.7

ROS Kinetic

Gazebo 7

### Overview
---

initially we are given a point cloud in the sensor_msgs::PointCloud2 format of the following scene:

![alt text][image1]


code flow during callback is as follows:

we need to convert the sensor_msgs::PointCloud2 to a pcl::PCLPointCloud2 data type to perform calculations using the pcl library. We can use the conversion functions provided in the `<pcl_conversions/pcl_conversions.h>` lib:

      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);


      // Convert to PCL data type
      pcl_conversions::toPCL( * cloud_msg, * cloud );


Since the intention is to perform cluster based segmentation, we can use voxel grid filtering to condense the data without a large loss of accuracy.

      // Perform voxel grid downsampling filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.01, 0.01, 0.01);
      sor.filter (* cloudFilteredPtr);

Resulting in the following pointcloud:

![alt text][image2]


For this example, we wish to focus on a region of interest in the z axis range ``(.5-1.1)``. This can be accomplished using the passthrough filter function of the class.

      passthrough = cloud_filtered.make_passthrough_filter()

      # specify the axis and range
      axis_field = 'z'
      range_low = 0
      range_high = 2
      # update the passthrough object
      passthrough.set_filter_field_name(z)
      passthrough.set_filter_limits(range_low,range_high)
      # call the filter function and save the results to the cloud_filtered instance
      cloud_filtered = passthrough.filter()

resulting in the following point cloud:

![alt text][image3]


In the simulation environment, objects are placed on a flat planar table surface. Since we wish to segment objects on the surface, we can remove the table from the could. Since the table is planar, we can use the RANSAC geometric filtration algorithm and extract the outliers to remove points corresponding to the table face.   

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
      seg1.setDistanceThreshold (0.01);

      seg1.setInputCloud (xyzCloudPtrFiltered);
      seg1.segment (* inliers, * coefficients);


      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;

      //extract.setInputCloud (xyzCloudPtrFiltered);
      extract.setInputCloud (xyzCloudPtrFiltered);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (* xyzCloudPtrRansacFiltered);

![alt text][image4]    

From the above point cloud, its apparent that the RANSAC algorithm has left the table edge since its not planar with the table face. Another passthrough filter removing all points below the table height should get rid of the edge.


      // create a pcl object to hold the passthrough filtered results
      pcl::PointCloud<pcl::PointXYZRGB> * xyz_cloud_filtered_passthrough = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrPassthroughFiltered (xyz_cloud_filtered_passthrough);

      // Create the filtering object
      pass.setInputCloud (xyzCloudPtrRansacFiltered);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits ((xyzCloudPtrFiltered->points[inliers->indices[0]].z +.01 ), (xyzCloudPtrFiltered->points[inliers->indices[0]].z  + 5));
      pass.filter (* xyzCloudPtrPassthroughFiltered);

![alt text][image5]


with the edge removed, we can use Euclidean cluster segmentation to identify each unique cluster.



      // Create the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (xyzCloudPtrPassthroughFiltered);

      // create the extraction object for the clusters
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
      // specify euclidean cluster parameters
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (xyzCloudPtrPassthroughFiltered);
      // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
      ec.extract (cluster_indices);

After computation, a different color is assigned to each cluster for visualization purposes.

![alt text][image6]

At this point, image identification could be used on each pcl cluster to locate an object of interest.

Total computation is low enough for real time execution, performing at ~5fps. The following is ROS_INFO logging ros::Time to console before the final point cloud is published. 

![alt text][image7]
