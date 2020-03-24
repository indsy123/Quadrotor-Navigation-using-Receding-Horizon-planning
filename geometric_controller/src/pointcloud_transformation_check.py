#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
new script for trajectory generation in camera FOV from scratch
"""
import time, itertools, math
import numpy as np
from math import sqrt
#import networkx as nx
#from gudhi import *
#import gudhi 
#import quaternion
from pyquaternion import Quaternion as Qt
from scipy.spatial import cKDTree
from scipy.special import erf
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#from polynomial_trajectory_generation_newlogic import Minimum_jerk_trajetory
#from scipy.interpolate import interp1d
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs
#from collections import OrderedDict
#from itertools import compress 
from nav_msgs.msg import Odometry
#from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Point, Quaternion, PoseStamped
#from geometric_controller.msg import PolynomialCoefficients



class test(object): 
    def __init__(self):
        """initialization"""
        self.path = '/home/pelican/data_collection/'
        open(self.path+'occ_points.txt', 'w').close()
        self.counter = 0
        
        self.Rcg_vibase = np.array([[1.0, 0.0, 0.0, 0.12], [0.0, 1.0, 0.0, 0.04], [0.0, 0.0, 1.0, -0.0], [0.0, 0.0, 0.0, 1.0]])
        self.Rcdoc_cdl = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        self.Rvibase_cl = np.array([[1, 0, 0, 0.0], [0, 1, 0, 0.015], [0, 0, 1, 0.0], [0, 0, 0, 1]])
        #self.goal = np.array([9.5,0,1])

        self.pcl_pub = rospy.Publisher('/udrone1/d435/cloud_for_trajectory', PointCloud2, queue_size = 1, tcp_nodelay = True)

        rospy.Subscriber('/udrone1/d435/depth_registered/voxel_points', PointCloud2, self.transform_callback, tcp_nodelay = True)
        #rospy.Subscriber('/udrone1/modifiedodom', Odometry, self.odom_callback, tcp_nodelay = True)
    
    def odom_callback(self, data):
        #print 'in the callback'
        self.odom_time = data.header.stamp.to_sec()
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = self.q.rotation_matrix
        V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.Vglobal = np.dot(self.Rglobal, V)
        #rpy = self.quaternion_to_euler(self.q[1], self.q[2], self.q[3], self.q[0])
        #self.yaw = rpy[0]
        #vector = np.array([self.goal[0]-self.Pglobal[0], self.goal[1]-self.Pglobal[1], self.goal[2]-self.Pglobal[2]])
        #self.directionvector = vector/np.linalg.norm(vector)

            



    def transform_callback(self, data): 
        """main motion planner, plan different trajectories for different stages"""
        a = time.time()
        
        occupied_points = ros_numpy.numpify(data)
  
        
        
        #occ = np.zeros((occupied_points.shape[0],3))
        #occ[:,0] = occupied_points['x']
        #occ[:,1] = occupied_points['y']
        #occ[:,2] = occupied_points['z']
        
        #occ = np.delete(occ, np.argwhere(occ[:,2] <=0).ravel(), axis=0)
        
        #p1 = np.dot(self.Rcdoc_cdl[0:3,0:3], occ.T).T
        #p2 = self.Rvibase_cl[0:3, 3:4].T + p1
        #p3 = self.Rcg_vibase[0:3, 3:4].T + np.dot(self.Rcg_vibase[0:3, 0:3].T, p2.T).T 
        #occ_p = self.Pglobal[np.newaxis] + np.dot(self.Rglobal, p3.T).T
        if self.counter ==300:
            for i in occupied_points:         
                f = open(self.path+'occ_points.txt','a')
                f.write('%s, %s\n' %(data.header.stamp.to_sec(),i))
            
        #print '1', time.time()-a
        # publish generated pointcloud
        #header = std_msgs.msg.Header()
        #header.stamp = data.header.stamp
        #header.frame_id = 'map'#'firefly1/vi_sensor/camera_depth_optical_center_link'
        #p = pc2.create_cloud_xyz32(header, occ_p)
        #self.pcl_pub.publish(p) 
        print '2', time.time()-a
        self.counter +=1
# if python says run, then we should run
if __name__ == '__main__': 
    rospy.init_node('pc_transformation_test', anonymous=True, log_level=rospy.DEBUG)
    
    f = test()

    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
       pass


