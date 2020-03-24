cmake_minimum_required(VERSION 2.8.3)
project(realsense_pointcloud_analysis)
find_package(PCL 1.7.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  octomap_ros
  octomap_msgs
  message_generation
)
find_package(CGAL REQUIRED  pcl_ros)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   publish_bounding_boxes.msg
   box_coordinates.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES realsense_pointcloud_analysis 
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  src
  external
  ${catkin_INCLUDE_DIRS}
  #include/${PROJECT_NAME}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

#add_library(${PROJECT_NAME} src/filter_realsense_cloud.cpp)
#  src/kinect_pc_transform.cpp
#)
add_executable(filter_cloud_setminpointsperpixel src/filter_cloud_setminpointsperpixel.cpp)
target_link_libraries(filter_cloud_setminpointsperpixel ${catkin_LIBRARIES})


#add_executable(${PROJECT_NAME} src/get_octree_data.cpp)
#target_link_libraries(get_octree_data ${catkin_LIBRARIES})

#target_link_libraries(kinect_pc_stiched
#  realsense_pointcloud_analysis
#  ${catkin_LIBRARIES}
#)

#add_executable(kinect_pc_transform src/kinect_pc_transform.cpp)
#target_link_libraries(kinect_pc_transform
#  realsense_pointcloud_analysis
#  ${catkin_LIBRARIES}
#)
 
link_directories(${PCL_LIBRARY_DIRS})

#target_link_libraries (${PROJECT_NAME}
#	${PCL_LIBRARIES}
#	${catkin_LIBRARIES}
#	${CGAL_LIBRARY}
#)

#target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARY})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

