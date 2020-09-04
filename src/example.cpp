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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
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
    boxFilter.setMin(Eigen::Vector4f(-2, -0.5, 0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(2, 0.50, 4, 1.0));
    boxFilter.setInputCloud(transformed_cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_box_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

    boxFilter.filter(*cloud_box_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud_box_filtered);
    sor.setMeanK (10);
    sor.setStddevMulThresh (0.5);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr statistical_ouitlier_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    sor.filter (*statistical_ouitlier_filtered);



    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (statistical_ouitlier_filtered);


    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance (0.15); // 15cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (statistical_ouitlier_filtered);
    ec.extract (cluster_indices);



    pcl::toPCLPointCloud2 (*statistical_ouitlier_filtered, *cloud_transformed);
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