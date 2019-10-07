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

#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <string>
//#include <gudhi/gudhi.h>
//#include <gudhi/Alpha_complex.h>

ros::Publisher pub;
//typedef pcl::PointCloud<pcl::PointXYZ> cloud;
void octomap_data_callback(const octomap_msgs::OctomapConstPtr& msg)
{	
	//++messages_received_;
	//setStatus(StatusProperty::Ok, "Messages", QString::number(messages_received_) + " octomap messages received");
	//setStatusStd(StatusProperty::Ok, "Type", msg->id.c_str());
	//if(!checkType(msg->id)){
	//	setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
	//	return;
	//}
	//ROS_DEBUG("Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

	//OcTreeType* octomap = NULL;
	octomap_msgs::Octomap octomap_data(*msg);
	octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(octomap_data);
	//octomap_msgs::OcTree* my_map = (OcTree*)tree;
	//octomap_msgs::OcTree my_tree = *my_map;
	//if(tree){
	//	octomap = dynamic_cast<OcTreeType*>(tree);
		//if(!octomap){
		//	setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type.");
		//	}
	//}
	//else{
	//	setStatusStd(StatusProperty::Error, "Message", "Failed to deserialize the octree message.")
	//}
	//my_tree.writeBinary("my_tree.bt");
	std::size_t octree_resolution= tree->getResolution();
	std::string octree_type = tree->getTreeType();
	printf("the resolution of octree is: %f \n", octree_resolution);
	printf("the type of octree is: %s \n", octree_type);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "get_octree_data");
  ros::NodeHandle nh;
  ros::Subscriber sub=nh.subscribe("/octomap_binary",1, octomap_data_callback);
  //pub = nh.advertise<pcl::PCLPointCloud2>("/camera/processed_cloud", 1, true);
  ros::spin();
}
