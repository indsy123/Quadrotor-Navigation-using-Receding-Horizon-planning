#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace std;
//Eigen::Matrix<double,4,4> transform;
ros::Publisher pub1; ros::Publisher pub3; ros::Publisher pub4; ros::Publisher pub5;
tf::TransformListener *listener_center_bumper_ptr;
//string camera_depth_frame;
//void depthcallback1(const pcl::PCLPointCloud2ConstPtr& cloud)

void depthcallback1(const sensor_msgs::PointCloud2ConstPtr& cloud)
{	
	// define transform as StampedTransform
	tf::StampedTransform transform;
	
	// defne two intermediate pcl clouds
	pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
	//transform input cloud from sensor_msgs::PointCloud2 to pcl pointclouds defined above
	pcl::fromROSMsg(*cloud, cloud_in);
	//get transform between two frames
	listener_center_bumper_ptr->lookupTransform("k2", "k1", ros::Time(0), transform);
	
	//transform pointclouds
	pcl_ros::transformPointCloud(cloud_in, cloud_trans, transform);
	cloud_trans.header.frame_id="k2";  // specify the frame id
	//pcl::toPCLPointCloud2(*cloud_out, *transformed_cloud);
	//define ros msg type for pointcloud
	sensor_msgs::PointCloud2 transformed_cloud;
	pcl::toROSMsg(cloud_trans, transformed_cloud);
	pub1.publish(transformed_cloud);
}
void depthcallback3(const sensor_msgs::PointCloud2ConstPtr& cloud)
{	
	// define transform as StampedTransform
	tf::StampedTransform transform;
	
	// defne two intermediate pcl clouds
	pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
	//transform input cloud from sensor_msgs::PointCloud2 to pcl pointclouds defined above
	pcl::fromROSMsg(*cloud, cloud_in);
	//get transform between two frames
	listener_center_bumper_ptr->lookupTransform("k2", "k3", ros::Time(0), transform);
	
	//transform pointclouds
	pcl_ros::transformPointCloud(cloud_in, cloud_trans, transform);
	cloud_trans.header.frame_id="k2";  // specify the frame id
	//pcl::toPCLPointCloud2(*cloud_out, *transformed_cloud);
	//define ros msg type for pointcloud
	sensor_msgs::PointCloud2 transformed_cloud;
	pcl::toROSMsg(cloud_trans, transformed_cloud);
	pub3.publish(transformed_cloud);
}
void depthcallback4(const sensor_msgs::PointCloud2ConstPtr& cloud)
{	
	// define transform as StampedTransform
	tf::StampedTransform transform;
	
	// defne two intermediate pcl clouds
	pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
	//transform input cloud from sensor_msgs::PointCloud2 to pcl pointclouds defined above
	pcl::fromROSMsg(*cloud, cloud_in);
	//get transform between two frames
	listener_center_bumper_ptr->lookupTransform("k2", "k4", ros::Time(0), transform);
	
	//transform pointclouds
	pcl_ros::transformPointCloud(cloud_in, cloud_trans, transform);
	cloud_trans.header.frame_id="k2";  // specify the frame id
	//pcl::toPCLPointCloud2(*cloud_out, *transformed_cloud);
	//define ros msg type for pointcloud
	sensor_msgs::PointCloud2 transformed_cloud;
	pcl::toROSMsg(cloud_trans, transformed_cloud);
	pub4.publish(transformed_cloud);
}
void depthcallback5(const sensor_msgs::PointCloud2ConstPtr& cloud)
{	
	// define transform as StampedTransform
	tf::StampedTransform transform;
	
	// defne two intermediate pcl clouds
	pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
	//transform input cloud from sensor_msgs::PointCloud2 to pcl pointclouds defined above
	pcl::fromROSMsg(*cloud, cloud_in);
	//get transform between two frames
	listener_center_bumper_ptr->lookupTransform("k2", "k5", ros::Time(0), transform);
	
	//transform pointclouds
	pcl_ros::transformPointCloud(cloud_in, cloud_trans, transform);
	cloud_trans.header.frame_id="k2";  // specify the frame id
	//pcl::toPCLPointCloud2(*cloud_out, *transformed_cloud);
	//define ros msg type for pointcloud
	sensor_msgs::PointCloud2 transformed_cloud;
	pcl::toROSMsg(cloud_trans, transformed_cloud);
	pub5.publish(transformed_cloud);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "kinect_filtered_pointcloud");
  ros::NodeHandle nh;
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  listener_center_bumper_ptr=&listener_center_bumper;
  pub1 = nh.advertise<sensor_msgs::PointCloud2>("/final_c1", 1, true);
  pub3 = nh.advertise<sensor_msgs::PointCloud2>("/final_c3", 1, true);
  pub4 = nh.advertise<sensor_msgs::PointCloud2>("/final_c4", 1, true);
  pub5 = nh.advertise<sensor_msgs::PointCloud2>("/final_c5", 1, true);
  //ros::Subscriber sub1=nh.subscribe("/camera/depth_registered/points",1, depthcallback_realsense);
  ros::Subscriber sub1=nh.subscribe("/k1/sd/points",1, depthcallback1);
  ros::Subscriber sub3=nh.subscribe("/k3/sd/points",1, depthcallback3);
  ros::Subscriber sub4=nh.subscribe("/k4/sd/points",1, depthcallback4);
  ros::Subscriber sub5=nh.subscribe("/k5/sd/points",1, depthcallback5);
  
  ros::spin();
}
