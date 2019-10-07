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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher pub;
tf::TransformListener *listener_center_bumper_ptr;

void callback(const PointCloud2ConstPtr& c1, const PointCloud2ConstPtr& c2, const PointCloud2ConstPtr& c3,const PointCloud2ConstPtr& c4)
{	
	//sensor_msgs::PointCloud2 final_cloud;
	
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c, cloud_d, concatenated_cloud;
	pcl::fromROSMsg(*c1, cloud_a); pcl::fromROSMsg(*c2, cloud_b); pcl::fromROSMsg(*c3, cloud_c); 
	pcl::fromROSMsg(*c4, cloud_d); 
	//pcl::fromROSMsg(*c5, cloud_e);
	concatenated_cloud = cloud_a; concatenated_cloud += cloud_b; concatenated_cloud += cloud_c; 
	concatenated_cloud += cloud_d; 

	// new cloud that is voxel filtered

	
	pcl::PCLPointCloud2::Ptr voxel_inp(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(concatenated_cloud, *voxel_inp); // LINE 29!!
	//pcl::PCLPointCloud2 voxeled;
	pcl::PCLPointCloud2::Ptr voxeled (new pcl::PCLPointCloud2 ());

	//use voxel filter to reduce the size of the point cloud 
	pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
	voxel.setInputCloud (voxel_inp);
	voxel.setLeafSize (0.05, 0.05, 0.05);
	voxel.filter (*voxeled);

	//pcl::toROSMsg(voxeled, final_cloud);
	pub.publish(*voxeled);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "stiched_pointcloud");
  ros::NodeHandle nh;
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  listener_center_bumper_ptr=&listener_center_bumper;
  pub = nh.advertise<pcl::PCLPointCloud2>("/final_cloud", 1, true);
  //ros::Subscriber sub1=nh.subscribe("/camera/depth_registered/points",1, depthcallback_realsense);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub1(nh, "/final_c1", 1);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub2(nh, "/k2/sd/points", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub3(nh, "/final_c3", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub4(nh, "/final_c4", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub5(nh, "/final_c5", 1);
  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pc_sub1, pc_sub3, pc_sub4, pc_sub5);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  ros::spin();
}
