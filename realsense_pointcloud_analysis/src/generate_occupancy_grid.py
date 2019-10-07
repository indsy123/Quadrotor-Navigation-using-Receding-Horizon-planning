#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
test algrithm to generate an accupancy grid in simple case
unless otherwise specified all the dimension are in m and angles are in degrees
@author: indrajeet
"""

import numpy as np 
import rospy 
import matplotlib.pyplot as plt
import time
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointCloud
import pcl 
import ros_numpy
from pcl import PointCloud as pcl_cloud
import pcl_ros
import networkx as nx
from scipy.spatial.distance import pdist
from scipy import spatial 
import dionysus as d
import gudhi



class generate_uniform_3d_grid(object):
    """calculates the convex hull of few points"""
    def __init__(self): 
        """initializes parameters"""
        self.counter = 0
        self.camera_location = np.array([0,0,0])
        self.SensorRangeMax = 2.0
        self.SensorRangeMin = 0.15
        self.resolution = 0.05
        self.xFOV = 63.4/2 # as per realsense camera spec
        self.yFOV = 40.4/2 # as per realsennse camera spec
        
        xmax = self.SensorRangeMax*np.tan(self.xFOV*np.pi/180)
        ymax = self.SensorRangeMax*np.tan(self.yFOV*np.pi/180)
        xmin = self.SensorRangeMin*np.tan(self.xFOV*np.pi/180)
        ymin = self.SensorRangeMin*np.tan(self.yFOV*np.pi/180)
        
        NumberOfPointsX = int(2*xmax/self.resolution)+1
        NumberOfPointsY = int(2*ymax/self.resolution)+1
        
        self.x_farend = np.linspace(-xmax, xmax, NumberOfPointsX)
        self.y_farend = np.linspace(-ymax, ymax, NumberOfPointsY)
        self.x_nearend = np.linspace(-xmin, xmin, NumberOfPointsX)
        self.y_nearend = np.linspace(-ymin, ymin, NumberOfPointsY)
        #open("pointcloud_realsense.txt", "w").close()

        #xx, yy = np.meshgrid(self.x_farend, self.y_farend)
        #xf = xx.ravel(); yf = yy.ravel()
        #zf = np.ones(len(xf))* self.SensorRangeMax
        
        #Pgrid_farend=np.zeros((len(xf),3))
        #Pgrid_farend[:,0]=xf
        #Pgrid_farend[:,1]=yf
        #Pgrid_farend[:,2]=zf 

        # make a cloud of points on the near end of the camera 
        #xxx, yyy = np.meshgrid(self.x_nearend, self.y_nearend)
        #xn = xxx.ravel(); yn = yyy.ravel()
        #zn = np.ones(len(xn))* self.SensorRangeMin
        
        #Pgrid_nearend=np.zeros((len(xn),3))
        #Pgrid_nearend[:,0]=xn
        #Pgrid_nearend[:,1]=yn
        #Pgrid_nearend[:,2]=zn
        #self.total_points = np.concatenate((Pgrid_farend, Pgrid_nearend), 0)
        
        #print 'time1:',time.time()-self.start_time
        #for i in range(len(Pgrid_farend)): 
        #    xyz = np.linspace(Pgrid_nearend[i], Pgrid_farend[i], int(np.linalg.norm(Pgrid_farend[i]-Pgrid_nearend[i])/self.resolution)+1)
        #    self.total_points = np.concatenate((self.total_points, xyz), 0)        
        self.z = np.arange(self.SensorRangeMin, self.SensorRangeMax+0.01, self.resolution)
        self.start_time = time.time()
        try: 
            rospy.Subscriber('/camera/depth_registered/voxel_points', PointCloud2, self.callback, queue_size = 1, tcp_nodelay = True)
        except: 
            print 'problem subcribing to pointcloud'
            
    def get_edges(self, grid1, grid2): 
        combined_grid = np.concatenate((grid1, grid2), 0)
        c1 = pcl_cloud(np.array(grid1, dtype=np.float32)) 
        c_combined = pcl_cloud(np.array(combined_grid, dtype=np.float32))
        kdt = pcl.KdTreeFLANN(c_combined)
        indices, sqr_distances = kdt.nearest_k_search_for_cloud(c_combined, 8)
        
    
    def callback(self, data):
        
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'camera_color_optical_frame'
        """
        # make a cloud of points on the mfar end of the camera 
        xx, yy = np.meshgrid(self.x_farend, self.y_farend)
        xf = xx.ravel(); yf = yy.ravel()
        zf = np.ones(len(xf))* self.SensorRangeMax
        
        Pgrid_farend=np.zeros((len(xf),3))
        Pgrid_farend[:,0]=xf
        Pgrid_farend[:,1]=yf
        Pgrid_farend[:,2]=zf 

        # make a cloud of points on the near end of the camera 
        xxx, yyy = np.meshgrid(self.x_nearend, self.y_nearend)
        xn = xxx.ravel(); yn = yyy.ravel()
        zn = np.ones(len(xn))* self.SensorRangeMin
        
        Pgrid_nearend=np.zeros((len(xn),3))
        Pgrid_nearend[:,0]=xn
        Pgrid_nearend[:,1]=yn
        Pgrid_nearend[:,2]=zn
        total_points = np.concatenate((Pgrid_farend, Pgrid_nearend), 0)
        #print 'time1:',time.time()-self.start_time
        for i in range(len(Pgrid_farend)): 
            xyz = np.linspace(Pgrid_nearend[i], Pgrid_farend[i], int(np.linalg.norm(Pgrid_farend[i]-Pgrid_nearend[i])/self.resolution)+1)
            total_points = np.concatenate((total_points, xyz), 0)
        """
        # get points from the subscribed pointcloud
        print 'time0:', time.time()-self.start_time
        pc = ros_numpy.numpify(data)            
        Pcam = np.zeros((pc.shape[0],3))
        Pcam[:,0]=pc['x']
        Pcam[:,1]=pc['y']
        Pcam[:,2]=pc['z']
        PCcam = pcl_cloud(np.array(Pcam, dtype=np.float32))    
        
        final_grid = np.zeros((0,3))
        G = nx.Graph()
        for k in range(len(self.z)): 
            xmax = self.z[k]*np.tan(self.xFOV*np.pi/180); ymax = self.z[k]*np.tan(self.yFOV*np.pi/180)
            
            NumberOfPointsX = int(2*xmax/self.resolution)+1
            NumberOfPointsY = int(2*ymax/self.resolution)+1
            
            x = np.linspace(-xmax, xmax, NumberOfPointsX)
            y = np.linspace(-ymax, ymax, NumberOfPointsY)
            xx, yy = np.meshgrid(x, y)
            xface = xx.ravel(); yface = yy.ravel()
            zface = np.ones(len(xface))* self.z[k]
            
            Pgrid = np.zeros((len(xface),3))
            Pgrid[:,0]=xface
            Pgrid[:,1]=yface
            Pgrid[:,2]=zface
            final_grid = np.concatenate((final_grid, Pgrid), 0)
            
            #if k == 0:
            #    G.add_nodes_from(Pgrid)
            #else:
            #    G.add_nodes_from(Pgrid)
            #    PC_one_faces = pcl_cloud(np.array(Pgrid, dtype=np.float32))
            #    P_two_faces = np.concatenate((self.points_on_previous_face, Pgrid), 0)
            #    PC_two_faces = pcl_cloud(np.array(P_two_faces, dtype=np.float32))
            #    kdt_two_faces = pcl.KdTreeFLANN(PC_two_faces)
            #    i, d = kdt_two_faces.nearest_k_search_for_cloud(PC_one_faces, 10)
            #self.points_on_previous_face = Pgrid

        PCgrid = pcl_cloud(np.array(final_grid, dtype=np.float32))
        kdt = pcl.KdTreeFLANN(PCgrid)
        
        indices, sqr_distances = kdt.nearest_k_search_for_cloud(PCcam, 10)
        Pgrid_modified = np.delete(final_grid, indices, axis=0)  
        p = pc2.create_cloud_xyz32(header, Pgrid_modified)
       
        final_pub.publish(p)
        print 'time4:', time.time()-self.start_time

        
if __name__ == '__main__':
    rospy.init_node('generate_uniform_3d_voxelgrid', anonymous=True, log_level=rospy.DEBUG)
    final_pub = rospy.Publisher('/camera/depth_registered/free_voxel_points', PointCloud2, queue_size = 1)
    r = rospy.Rate(30)    
    traj = generate_uniform_3d_grid()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
