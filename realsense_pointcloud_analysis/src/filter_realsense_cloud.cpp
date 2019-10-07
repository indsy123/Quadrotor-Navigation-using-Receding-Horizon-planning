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
#include <string.h>
//#include <gudhi/gudhi.h>
//#include <gudhi/Alpha_complex.h>

using namespace std;
ros::Publisher pub;
//typedef pcl::PointCloud<pcl::PointXYZ> cloud;
void depthcallback(const pcl::PCLPointCloud2ConstPtr& cloud)
{	
	float xrange = 1.0472f;
	float yrange = 0.785f;
	float zrange = 2.5f;
	float leafsize = 0.05f;
	pcl::PCLPointCloud2::Ptr voxeled (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr passthroughx (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr passthroughy (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr passthroughz (new pcl::PCLPointCloud2 ());
	//geometry_msgs::Point32 point;
	
        //pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        //pcl::fromPCLPointCloud2(*cloud, cloud_filtered);
	
	//use voxel filter to reduce the size of the point cloud 
	pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
	voxel.setInputCloud (cloud);
	voxel.setLeafSize (leafsize, leafsize, leafsize);
	voxel.filter (*voxeled);

	// use PassThroguh filter to remove the points that are outside a range (in x, y and z directions)
	pcl::PassThrough<pcl::PCLPointCloud2> passx;
	passx.setInputCloud (voxeled);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-xrange, xrange);
        passx.filter (*passthroughx);

	pcl::PassThrough<pcl::PCLPointCloud2> passy;
	passy.setInputCloud (passthroughx);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (-yrange, yrange);
        passy.filter (*passthroughy);

	pcl::PassThrough<pcl::PCLPointCloud2> passz;
	passz.setInputCloud (passthroughy);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (0.0, zrange);
        passz.filter (*passthroughz);


	//pass.setFilterFieldName ("y");
	//pass.setFilterLimits (-yrange, yrange);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (0.0, zrange);
	//pass.filter (*passthrough);

	//add dummy points to the pointcloud
	//pcl::PointCloud<pcl::PointXYZ> filtered_cloud, new_cloud;
	//pcl::fromROSMsg(*passthrough, cloud_a);


	//passthrough.push_back (point); 


	pub.publish(passthroughz);

	// convert the PCLPointCloud2 message to sensor_msgs::PointCloud2, I thought it was needed for ROS as I have  got some errors initially but now script works without it
	//sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCL(*passthrough, output);
	//pub.publish(output);

	//pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud_filtered);

	/*//code snippet for point cloud visualization, not needed now, I can do that in rviz
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	viewer.showCloud (new_cloud);
   	while (!viewer.wasStopped ())
   	{
   	}
   	*/

	//convert the filtered point cloud (PCLPointCloud2 form) to the pcl::PointXYZ form to the x, y, z values
	//pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::fromPCLPointCloud2(*cloud_filtered,*new_cloud);

	/*// this is a working block of the code to store the point cloud data into a geometry_msgs/Point vector
	for(int i = 0 ; i < output->points.size(); ++i)
	{
		point.x = output->points[i].x;
		point.y = output->points[i].y;
		point.z = output->points[i].z;
		printf("%f, %f, %f\n", point.x, point.y, point.z);
	}
	*/
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "filtered_realsense_pointcloud");
  ros::NodeHandle nh;
  int no;  
  ros::param::get("number", no);
  string str = to_string(no);
  //printf("the value of quad number is: %s", str);
  //ros::Subscriber sub=nh.subscribe("/camera/depth_registered/points",1, depthcallback);
  //pub = nh.advertise<pcl::PCLPointCloud2>("/camera/depth_registered/voxel_points", 1, true);
  ros::Subscriber sub=nh.subscribe("/firefly"+str+"/vi_sensor/camera_depth/depth/points",1, depthcallback);
  pub = nh.advertise<pcl::PCLPointCloud2>("/firefly"+str+"/vi_sensor/camera_depth/depth/voxel_points", 1, true);
  ros::spin();
}
