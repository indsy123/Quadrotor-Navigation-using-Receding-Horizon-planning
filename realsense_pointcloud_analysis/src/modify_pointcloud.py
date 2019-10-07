#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 14:58:59 2019
modify the pointcloud
@author: ind
"""
import numpy as np 
import rospy 
import matplotlib.pyplot as plt
import time
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointCloud
from nav_msgs.msg import Odometry
import pcl 
import ros_numpy
from pcl import PointCloud as pcl_cloud
import pcl_ros

class modify_pointcloud(object):
    def __init__(self): 
        """initializes various paramters"""
        self.counter = 0
        self.camera_location = np.array([0,0,0])
        self.SensorRange = 2.5
        resolution = 0.05
        xFOV = 63.4/2 # as per realsense camera spec
        yFOV = 40.4/2 # as per realsennse camera spec
        xmax = self.SensorRange*np.tan(xFOV*np.pi/180)
        ymax = self.SensorRange*np.tan(yFOV*np.pi/180)
        self.start_time = time.time()
        self.x = np.linspace(-xmax, xmax, int(2*xmax/resolution)+1)
        self.y = np.linspace(-ymax, ymax, int(2*ymax/resolution)+1)
        
        self.pcl_pub = rospy.Publisher('/firefly/vi_sensor/camera_depth/depth/cloud_for_octomap', PointCloud2, queue_size = 1, tcp_nodelay = True)
        try: 
            #rospy.Subscriber('/camera/depth_registered/voxel_points', PointCloud2, self.callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/firefly/vi_sensor/camera_depth/depth/voxel_points', PointCloud2, self.cloud_callback, queue_size = 1, tcp_nodelay = True)
            
        except: 
            print 'problem subcribing to pointcloud'

    

    def cloud_callback(self, data):
        """process the cloud"""
        # get points from the subscribed pointcloud
        # for some reason storing pc into Pcam is working with point cloud that is voxel filtered. Investigate!! 
        pc = ros_numpy.numpify(data)  
        Pcam = np.zeros((pc.shape[0],3))
        Pcam[:,0] = pc['x']
        Pcam[:,1] = pc['y']
        Pcam[:,2] = pc['z']
  
        # make a cloud of points on the max z face of the camera 
     
        xx, yy = np.meshgrid(self.x, self.y)
        xvalues = xx.ravel(); yvalues = yy.ravel()
        zvalues = np.ones(len(xvalues)) * self.SensorRange
        
        Pgrid = np.zeros((len(xvalues),3))
        Pgrid[:,0] = xvalues
        Pgrid[:,1] = yvalues
        Pgrid[:,2] = zvalues 

        # project existing points to the plane containing new points
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        #header.frame_id = 'camera_color_optical_frame'
        
        #total_points = np.concatenate((Pcam, Pgrid), 0)
        #p = pc2.create_cloud_xyz32(header, total_points)
        
        if len(Pcam) != 0: 
            Pprojected = self.camera_location + np.array([(Pcam[i]-self.camera_location)*self.SensorRange/Pcam[i,2] for i in range(len(Pcam))])
            #projected_points = self.camera_location + map(lambda i, j : (i-self.camera_location)*self.SensorRange/j, points, points[:,2])
    
            PCprojected = pcl_cloud(np.array(Pprojected, dtype=np.float32))
            PCgrid = pcl_cloud(np.array(Pgrid, dtype=np.float32))
            kdt = pcl.KdTreeFLANN(PCgrid)
            
            indices, sqr_distances = kdt.nearest_k_search_for_cloud(PCprojected, 10)
            Pgrid_modified = np.delete(Pgrid, indices, axis=0)

            total_points = np.concatenate((Pcam, Pgrid_modified), 0)        
            p = pc2.create_cloud_xyz32(header, total_points)

        else: 
            #total_points = np.concatenate((Pcam, Pgrid), 0)
            p = pc2.create_cloud_xyz32(header, Pgrid)
            
        self.pcl_pub.publish(p)

if __name__ == '__main__':
    rospy.init_node('modify_pointcloud_for_octomap', anonymous=True, log_level=rospy.DEBUG)
    #pcl_pub = rospy.Publisher('/camera/depth_registered/cloud_for_octomap', PointCloud2, queue_size = 1)
    
    r = rospy.Rate(30)    
    traj = modify_pointcloud()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
