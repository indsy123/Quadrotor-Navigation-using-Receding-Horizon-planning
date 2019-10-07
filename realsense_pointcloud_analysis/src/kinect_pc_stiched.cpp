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
#include <gudhi/Alpha_complex.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher pub;
tf::TransformListener *listener_center_bumper_ptr;

void callback(const PointCloud2ConstPtr& c1, const PointCloud2ConstPtr& c2, const PointCloud2ConstPtr& c3, const PointCloud2ConstPtr& c4,const PointCloud2ConstPtr& c5)
{	
	sensor_msgs::PointCloud2 final_cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c, cloud_d, cloud_e, concatenated_cloud;
	pcl::fromROSMsg(*c1, cloud_a); pcl::fromROSMsg(*c2, cloud_b); pcl::fromROSMsg(*c3, cloud_c); 
	pcl::fromROSMsg(*c4, cloud_d); pcl::fromROSMsg(*c5, cloud_e);
	concatenated_cloud = cloud_a; concatenated_cloud += cloud_b; concatenated_cloud += cloud_c; 
	concatenated_cloud += cloud_c; concatenated_cloud += cloud_e;
	

	pcl::toROSMsg(concatenated_cloud, final_cloud);
	pub.publish(final_cloud);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "filtered_pointcloud");
  ros::NodeHandle nh;
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  listener_center_bumper_ptr=&listener_center_bumper;
  pub = nh.advertise<sensor_msgs::PointCloud2>("/final_cloud", 1, true);
  //ros::Subscriber sub1=nh.subscribe("/camera/depth_registered/points",1, depthcallback_realsense);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub1(nh, "/final_c1", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub2(nh, "/k2/sd/points", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub3(nh, "/final_c2", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub4(nh, "/final_c3", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub5(nh, "/final_c4", 1);
  TimeSynchronizer<PointCloud2, PointCloud2, PointCloud2, PointCloud2, PointCloud2> sync(pc_sub1, pc_sub2, pc_sub3, pc_sub4, pc_sub5, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
  ros::spin();
}
