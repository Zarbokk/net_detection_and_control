#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
// Container for original & filtered data
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    //transform_1 (0,0) = 0;
    //transform_1 (1,1) = 0;
    //transform_1 (2,2) = 0;
    //transform_1 (0,2) = 1;
    //transform_1 (1,0) = -1;
    //transform_1 (2,1) = -1;


    //transform_1 (0,1) = 0
    //transform_1 (1,0) = sin (theta);
    //transform_1 (1,1) = std::cos (theta);
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2* cloud_transformed = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    // Executing the transformatio
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());

    //copy cloud to source cloud
    pcl::fromPCLPointCloud2(*cloud,*source_cloud);
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1);




    //std::cout << "someString1" << "\n";

    // Create the filtering object

    //pcl::PassThrough<pcl::PCLPointCloud2> pass;
    //pass.setInputCloud (cloudPtr);
    //pass.setFilterFieldName ("x");
    //pass.setFilterLimits (0.0, 2.0);
    //pass.setFilterFieldName("y");
    //pass.setFilterLimits (0.0, 2.0);
    //pass.filter (cloud_filtered);
    // Convert to ROS data type

    pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-1, -0.3, 0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(1, 0.3, 4, 1.0));
    boxFilter.setInputCloud(transformed_cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_box_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

    boxFilter.filter(*cloud_box_filtered);







    pcl::toPCLPointCloud2 (*cloud_box_filtered, *cloud_transformed);
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_transformed);
    sensor_msgs::PointCloud2 output;


    pcl_conversions::fromPCL(*cloud_transformed, output);
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
  ros::Subscriber sub = nh.subscribe ("/d435i/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}