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
#import time
import numpy as np
from math import sqrt
#import networkx as nx
#from gudhi import *
#import gudhi 
#import scipy
from pyquaternion import Quaternion as Qt
#from scipy.spatial import cKDTree
#from scipy.special import erf
from polynomial_trajectory_generation_from_waypoints import Minimum_snap_trajetory
import rospy 
#import ros_numpy
#from sensor_msgs.msg import PointCloud2
#from std_msgs.msg import Bool
import std_msgs.msg
#from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Point, Quaternion
from nav_msgs.msg import Odometry 
from geometric_controller.msg import PolynomialCoefficients
#import sensor_msgs.point_cloud2 as pc2
#from collections import OrderedDict

class trajectory_from_waypoints(object): 
    def __init__(self, name, number, ip, io, direction, h, ht, vel, goal, order, vel_hl):

        """initialization"""
        self.uav = name
        self.no = str(number)
        self.uav_velocity = vel
        self.initial_position = np.asarray(ip)
        self.initial_orientation = np.asarray(io)
        self.desired_direction = np.asarray(direction)
        self.hovering_height = h
        self.initial_hover_time = ht
        self.end_point = np.asarray(goal)#np.array([7, -10, 1.0]) # this is the goal
        #self.target_velocity = 1.0
        self.trajectory_time = 0.0 # receding horizon planning time, keeps track of the time for the trajectory
        self.velocity_during_hover_land = vel_hl

        self.Pglobal = np.asarray(ip)
        self.Vglobal = np.array([0,0,0])
        self.Rglobal = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.N = order
        
        self.waypoint_counter = 0
        self.hover_counter = 0
        self.land_counter = 0
        self.trajectory_counter = 0
        
        self.reached_goal = False



        self.traj_polycoeff = rospy.Publisher('/'+self.uav+self.no+'/polynomial_coefficients', PolynomialCoefficients, queue_size = 1, tcp_nodelay = True)
        try:
            rospy.Subscriber('/'+self.uav+self.no+'/modifiedodom', Odometry, self.planning_callback, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the topic above')
    

    def odometry_callback(self, data):
        """get the odometry of the quadrotor"""
        
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = q.rotation_matrix
        V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.Vglobal = np.dot(self.Rglobal, V)
        if self.odometry_counter == 0: 
            self.initial_position = self.Pglobal
        self.odometry_counter += 1
        
    def dist(self, p1,p2):
        return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
    
        
            
    def generate_trajectory_from_waypoints(self, data):
        """generates trajecory from the way points""" 
        self.trajectory_time = data.header.stamp.to_sec()-self.hover_start_time

        if self.waypoint_counter == 0 and self.reached_goal == False:
            # -------change this segment in order to construct  a new trajectory---------------
            start = self.Pglobal
            end = self.end_point
            no_of_segments = 8
            path  = []
            for j in range(no_of_segments+1): 
                path.append(list(start + j * (end-start)/no_of_segments))
            path = zip(*path)
	    # -------change this segment in order to construct  a new trajectory---------------

            waypoint_specified = True
            waypoint_bc = False           
            velocity = self.uav_velocity
            planning_counter = 0
            vel = np.array([[0,0,0]]); acc = np.array([[0,0,0]]); jerk = np.array([[0,0,0]])
            T, _p1, _p2, _p3 = Minimum_snap_trajetory(planning_counter, velocity, path, waypoint_specified, waypoint_bc, vel, acc, jerk).construct_polynomial_trajectory()
            
     
            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp = data.header.stamp#rospy.Time.now()
            header.frame_id = 'world'
            msg.header = header
            msg.polynomial_order = self.N
            for i in range(len(_p1)): 
                msg.poly_x.append(_p1[i]); msg.poly_y.append(_p2[i]); msg.poly_z.append(_p3[i])
            msg.number_of_segments = no_of_segments
            msg.planning_start_time = str(data.header.stamp.to_sec())  
            for j in T[:no_of_segments+1]: 
                msg.segment_end_times.append(j)
            msg.desired_direction.x = self.desired_direction[0]; msg.desired_direction.y = self.desired_direction[1]; msg.desired_direction.z = self.desired_direction[2]
            msg.trajectory_mode = 'planning_from_waypoints'
            self.traj_msg = msg
            self.trajectory_end_in_global_time = self.trajectory_time + T[-1]
            
 
            
                
        self.traj_polycoeff.publish(self.traj_msg)
        # check if the goal is reached
        if self.trajectory_time >= self.trajectory_end_in_global_time + 5.0: # 5 sec is the time for which the quad will hover before coming down
            self.reached_goal = True
       
        self.waypoint_counter += 1

    def hover_up(self, data): 
        """give commands to hover at 1m above the starting position"""

        if self.hover_counter == 0: 
            self.hover_start_time = data.header.stamp.to_sec() 
	    #print 'i m here--', self.hover_start_time     
        self.trajectory_time = data.header.stamp.to_sec()-self.hover_start_time
        #print "hover start time and trajectory time is", self.hover_start_time, self.trajectory_time

        if self.hover_counter == 0:
            waypoint_specified = True
            waypoint_bc = False
            path = [[self.initial_position[0], self.initial_position[1], self.initial_position[2]], [self.initial_position[0], self.initial_position[1], self.hovering_height]]
            path = zip(*path)
            vel = np.array([[0,0,0]]); acc = np.array([[0,0,0]]); jerk = np.array([[0,0,0]])
            planning_counter = -1; velocity = self.velocity_during_hover_land
            T, p1, p2, p3 = Minimum_snap_trajetory(planning_counter, velocity, path, waypoint_specified, waypoint_bc, vel, acc, jerk).construct_polynomial_trajectory()

            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp =  data.header.stamp
            header.frame_id = 'world'
            msg.header = header
            msg.polynomial_order = self.N
            for i in range(len(p1)): 
                msg.poly_x.append(p1[i]); msg.poly_y.append(p2[i]); msg.poly_z.append(p3[i])
            msg.number_of_segments = 1
            msg.planning_start_time = str(self.hover_start_time)
            for j in T: 
                msg.segment_end_times.append(j)
            msg.desired_direction.x = self.desired_direction[0] 
            msg.desired_direction.y = self.desired_direction[1]
            msg.desired_direction.z = self.desired_direction[2]
            msg.trajectory_mode = 'hover'
            self.hover_traj_msg = msg
	    
	    
	#print self.hover_traj_msg
        self.traj_polycoeff.publish(self.hover_traj_msg)
        self.hover_counter += 1
        #self.hover_end_time = data.header.stamp.to_sec()


    def land(self, data): 
        """generate trajectory for landing"""

        if self.land_counter == 0: 
            self.land_start_time = data.header.stamp.to_sec()
            self.land_start = [self.Pglobal[0], self.Pglobal[1], self.Pglobal[2]]

        if self.land_counter == 0: 
            waypoint_specified = True
            waypoint_bc = False
            path = [[self.land_start[0], self.land_start[1], self.land_start[2]], [self.land_start[0], self.land_start[1], 0.12]]
            path = zip(*path)
            vel = np.array([[0,0,0]]); acc = np.array([[0,0,0]]); jerk = np.array([[0,0,0]])
            planning_counter = -1; velocity = self.velocity_during_hover_land
            T, p1, p2, p3 = Minimum_snap_trajetory(planning_counter, velocity, path, waypoint_specified, waypoint_bc, vel, acc, jerk).construct_polynomial_trajectory()

            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp =  data.header.stamp
            header.frame_id = 'world'
            msg.header = header
            msg.polynomial_order = self.N
            for i in range(len(p1)): 
                msg.poly_x.append(p1[i]); msg.poly_y.append(p2[i]); msg.poly_z.append(p3[i])
            msg.number_of_segments = 1
            msg.planning_start_time =  str(self.land_start_time)
            for j in T: 
                msg.segment_end_times.append(j)
    
            msg.desired_direction.x = self.desired_direction[0]; msg.desired_direction.y = self.desired_direction[1]; msg.desired_direction.z = self.desired_direction[2]
            msg.trajectory_mode = 'land'
            self.land_traj_msg = msg
            
        self.traj_polycoeff.publish(self.land_traj_msg)
        self.land_counter += 1    
        
    def planning_callback(self, data): 
        """main motion planner, plan different trajectories for different stages"""
        # get positions
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = q.rotation_matrix
        V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.Vglobal = np.dot(self.Rglobal, V)
        if self.trajectory_counter == 0: 
            self.initial_position = self.Pglobal
            
        #start generating the trajectory
        if self.trajectory_time <= self.initial_hover_time and self.reached_goal == False: 
            self.hover_up(data)
        elif self.trajectory_time > self.initial_hover_time and self.reached_goal == False:
            self.generate_trajectory_from_waypoints(data)
        elif self.trajectory_time > self.initial_hover_time and self.reached_goal == True: 
            self.land(data)


	# to only hover and land
        #if self.trajectory_time <= self.initial_hover_time: 
        #    self.hover_up(data)
        #else: 
        #    self.land(data)

        self.trajectory_counter += 1

# if python says run, then we should run
if __name__ == '__main__':
    
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    initial_position = rospy.get_param('initial_position')
    initial_orientation = rospy.get_param('initial_orientation')
    direction = rospy.get_param('desired_direction')
    h = rospy.get_param('initial_hovering_height')
    ht = rospy.get_param('initial_hovering_time')
    vel = rospy.get_param('uav_velocity')
    goal = rospy.get_param('goal_point')
    order = rospy.get_param('polynomial_order')
    vel_hl = rospy.get_param('velocity_during_hover_land')
        
    rospy.init_node('trajectory_from_waypoints', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(30)    
    traj = trajectory_from_waypoints(name, number, initial_position, initial_orientation, direction, h, ht, vel, goal, order, vel_hl)
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
    
