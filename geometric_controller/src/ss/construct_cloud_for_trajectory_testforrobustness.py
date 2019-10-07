#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rrt.py
This program generates a simple rapidly
exploring random tree (RRT) in a rectangular region. It is the modification
of the original using KDTree based NN search to speed up

Written by Steve LaValle
May 2011
Modified by: Indrajeet Yadav
April 2019
Modifications: NN search using KDTree and integrated to networkx to find 
shortest path using Dijkstra. It also adds few points
to simulate a pointcloud and avoid areas circling those points.
Modified to run RRT in a 3D cone of 45 degree. 

R1: generated the grid before-hand and removed the points corresponding to 
obstacles from the grid, to avoid generating random points in a loop and checking for 
colision.
R2: made a uniform initial grid not the cone but a parallelopiped. No
advantage computationally, neither the path looks good. 
R3: tried to use python pcl instead of cKDTree. no advantange so left halfway. 
R4: made a graph (using 1-simplices and pygudhi) instead of running RRT. 
R5: renamed and converted into a class structure

After converting to ROS node
R0: just starts from initial condition and plan the plan once without receding 
horizon planning 
R1: receding horizon planning, polynomials are generated for 5 segments but 
trajecotry is constracted for only 3 with starting point as the end of the 3rd
segment (along with the planned boundary conditions at this point)
TODO: how to make sure replanning time almost matches to this end of 3rd segment!

"""
import numpy as np
from math import sqrt
from pyquaternion import Quaternion as Qt
from scipy.spatial import cKDTree
import rospy 
import itertools
import ros_numpy
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from nav_msgs.msg import Odometry 
import sensor_msgs.point_cloud2 as pc2
from collections import OrderedDict

class construct_cloud_in_fov(object): 
    def __init__(self, name, number, res, radius, sr, fov_x, fov_y):

        """initialization"""
        self.uav = name
        self.no = str(number)
        self.resolution = res
        self.radius_of_obstacle = radius
        self.sensor_range = sr
        self.cone_angle_x = fov_x
        self.cone_angle_y = fov_y

        self.Pglobal = np.array([0,0,0])
        self.Vglobal = np.array([0,0,0])
        self.Rglobal = np.array([[1,0,0], [0,1,0], [0,0,1]])

        #self.Rcg_vibase = np.array([[0.8775826, 0.0, 0.4794255, 0.1], [0.0, 1.0, 0.0, 0.0], [-0.4794255, 0.0, 0.8775826, -0.03], [0.0, 0.0, 0.0, 1.0]]) #0.5 radian pitch
        self.Rcg_vibase = np.array([[1.0, 0.0, 0.0, 0.2], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -0.03], [0.0, 0.0, 0.0, 1.0]])
        # camera depth optical center link to camera depth link (changes xyz with z point forward to camera , to z pointing upwards)
        self.Rcdoc_cdl = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        # visensor base to camera left link
        self.Rvibase_cl = np.array([[1, 0, 0, 0.015], [0, 1, 0, 0.055], [0, 0, 1, 0.0065], [0, 0, 0, 1]])
        #self.Rvibase_cl = np.array([[1, 0, 0, 0.0], [0, 1, 0, 0.0], [0, 0, 1, 0.0], [0, 0, 0, 1]])
        # rotation matrix to transform points from z forward to z up
        #self.Rzforward_to_zup = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        self.pcl_pub = rospy.Publisher('/'+self.uav+self.no +'/cloud_for_trajectory', PointCloud2, queue_size = 1, tcp_nodelay = True)

        try:
            rospy.Subscriber('/'+self.uav+self.no+'/vi_sensor/camera_depth/depth/voxel_points', PointCloud2, self.point_generation_callback, tcp_nodelay = True)
            rospy.Subscriber('/'+self.uav+self.no+'/odometry_sensor1/odometry', Odometry, self.odometry_callback, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the topic above')
    

    def odometry_callback(self, data):
        """get the odometry of the quadrotor"""
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = q.rotation_matrix
        V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.Vglobal = np.dot(self.Rglobal, V)
        
    def dist(self, p1,p2):
        return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
    
    def generate_regular_pyramid_grid(self): 
        """generate regular grid inside the rectangular pyramid located at uav center and aligned with yaw"""
        z = np.arange(0, self.sensor_range+self.resolution, self.resolution)
        points_in_pyramid = np.zeros((0,3))

        for zz in z: 
            xmax = zz*np.tan(self.cone_angle_x/2); ymax = zz*np.tan(self.cone_angle_y/2)
            NumberOfPointsX = int(2*xmax/self.resolution)+3
            NumberOfPointsY = int(2*ymax/self.resolution)+3
            
            x = np.linspace(-xmax, xmax, NumberOfPointsX)
            y = np.linspace(-ymax, ymax, NumberOfPointsY)
            xx, yy = np.meshgrid(x, y)
            xface = xx.ravel(); yface = yy.ravel()
            zface = np.ones(len(xface))* zz
            
            Pgrid = np.zeros((len(xface),3))
            Pgrid[:,0] = xface
            Pgrid[:,1] = yface
            Pgrid[:,2] = zface
            points_in_pyramid = np.concatenate((points_in_pyramid, Pgrid), 0)
        return points_in_pyramid
        
        
    def generate_final_grid(self, points_in_cone, occupied_points): 
        """generate the final points in the cone by removing obstacles"""
 
        kdt_points_in_cone = cKDTree(points_in_cone)
        
   
        p = np.zeros((occupied_points.shape[0],3))
        p[:,0] = occupied_points['x']
        p[:,1] = occupied_points['y']
        p[:,2] = occupied_points['z']

        pp = [list(self.Rvibase_cl[0:3, 3:4].ravel() + x) for x in p]
        
        points_near_obstacle = kdt_points_in_cone.query_ball_point(pp, self.radius_of_obstacle)
        points_to_remove = list(itertools.chain.from_iterable(points_near_obstacle))
        list_without_duplicate_points = list(OrderedDict.fromkeys(points_to_remove))
        modified_points_in_cone = np.delete(points_in_cone, list_without_duplicate_points, 0)

        return modified_points_in_cone
            
    def point_generation_callback(self, data):
        """generates points in camera workspace""" 
        
        start_point = np.zeros((0,3))
        start_point = np.concatenate((start_point, self.Pglobal[np.newaxis]), 0) # was self.p_eoe

        points_in_cone = self.generate_regular_pyramid_grid() 
        occupied_points = ros_numpy.numpify(data)

        if len(occupied_points) != 0: 
            points_in_cone = self.generate_final_grid(points_in_cone, occupied_points)


        p1 = [np.dot(self.Rcdoc_cdl[0:3,0:3], x[np.newaxis].T) for x in points_in_cone]
        p2 = [self.Rcg_vibase[0:3, 3:4] + np.dot(np.linalg.inv(self.Rcg_vibase[0:3, 0:3]), x) for x in p1]
        points_in_cone = [self.Pglobal + np.dot(self.Rglobal, x).ravel() for x in p2]
        points_in_cone = [x for x in points_in_cone if not x[2] <= 0]

        points_in_cone = np.concatenate((start_point, points_in_cone), 0)
        
        # publish generated pointcloud
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        p = pc2.create_cloud_xyz32(header, points_in_cone)
        self.pcl_pub.publish(p)         
      

# if python says run, then we should run
if __name__ == '__main__':
    
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    res = rospy.get_param('visibility_graph_resolution')
    radius = rospy.get_param('uav_radius')
    sr = rospy.get_param('sensor_range')    
    fov_x = rospy.get_param('camera_fov_x')
    fov_y = rospy.get_param('camera_fov_y')
    rospy.init_node('construct_cloud_for_trajectory', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(30)    
    traj = construct_cloud_in_fov(name, number, res, radius, sr, fov_x, fov_y)
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
    
