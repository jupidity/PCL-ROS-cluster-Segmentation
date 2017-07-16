### ROS Node for Cluster Based Segmentation with PCL
---

`Python` `ROS` node for image segmentation on a cluttered environment with cluster based methods. Tested in simulation environment provided at https://github.com/udacity/RoboND-Perception-Exercises.git. Node sensor input is `ROS` msg `PointCloud2` generated in `Gazebo` simulation environment from RGBD camera. `pcl_helper.py` lib provided for `ROS` `PointCloud2` msg to PCL data structure conversions.

`ROS` node `segmentation.py` in ``/scripts`` directory.   


### Overview
---


code flow during callback is as follows:

The RGBD camera sends sensor data as an ROS msg of type PointCloud2 in the format

      [sensor_msgs/PointCloud2]:
      std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
      uint32 height
      uint32 width
      sensor_msgs/PointField[] fields
      uint8 INT8=1
      uint8 UINT8=2
      uint8 INT16=3
      uint8 UINT16=4
      uint8 INT32=5
      uint8 UINT32=6
      uint8 FLOAT32=7
      uint8 FLOAT64=8
      string name
      uint32 offset
      uint8 datatype
      uint32 count
      bool is_bigendian
      uint32 point_step
      uint32 row_step
      uint8[] data
      bool is_dense

We wish to do computations on the point cloud is the standard PointXYZRGB pcl data structure:

      [PointXYZRGB]:
      uint32 X
      uint32 Y
      uint32 Z
      uint32 RGB



For this application, we can use the data conversion functions provided in `pcl_helper.py` to convert the `ROS` received msg of type `PointCloud2` to a `pcl_XYZRGB` pointcloud

      cloud = pcl_to_ros(pcl_msg)

Since the intension is to perform cluster based segmentation, we can use voxel grid filtering to condense the data without a large loss of accuracy. Luckily, the `pcl` PointXYZRGB class has build in functions for voxel filtering we can use.

      vox = cloud.make_voxel_grid_filter()

      # Set the leaf size, here we use a cube w/ dimensions LEAF_SIZE
      LEAF_SIZE = 1     
      vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

      # Call the filter function to obtain the resultant downsampled point cloud
      cloud_filtered = vox.filter()

For this example, we wish to focus on a region of interest in the z axis range 0-2. This can be accomplished using the passthrough filter function of the PointXYZRGB class.

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

In the simulation environment, objects are placed on a flat planar table surface. Since we wish to segment objects on the surface, we can remove the table from the could. Since the table is planar, this is a perfect fit for the RANSAC geometric filtration algorithm.       
