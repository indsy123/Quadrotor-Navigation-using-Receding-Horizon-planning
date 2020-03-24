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
from polynomial_trajectory_generation_newlogic import Minimum_jerk_trajetory
#from scipy.interpolate import interp1d
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs
#from collections import OrderedDict
#from itertools import compress 
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Point, Quaternion, PoseStamped
from geometric_controller.msg import PolynomialCoefficients



class test(object): 
    def __init__(self, name, number, ip, io, res, radius, sr, fov_x, fov_y, h, vel, goal, ht, order, vel_hl):
        """initialization"""
        self.path = '/home/pelican/data_collection/'
        open(self.path+'occ_points.txt', 'w').close()
        open(self.path+'occ_points2.txt', 'w').close()
        open(self.path+'end_points.txt', 'w').close()
        open(self.path+'goals.txt', 'w').close()
        open(self.path+'velocities.txt', 'w').close()
        #self.uav = name
        no = str(number)
        self.uav_velocity = vel
        self.resolution = res
        self.safety_radius = radius
        self.sensor_range = sr
        self.cone_angle_x = fov_x
        self.cone_angle_y = fov_y
        self.vel_hl = vel_hl
        self.N = order+1
        self.initial_hover_time = ht 

        self.RHPendcounter = 0
        self.traj_gen_counter = 0
        self.hover_counter = 0
        self.land_counter = 0
        self.replanning_started = 0
        self.planning_counter = 0
        self.hover_start_time = time.time()
        self.hovering_time = time.time()
        self.initial_position = np.asarray(ip)
        self.initial_orientation = np.asarray(io)
        self.hovering_height = h
        self.goal = np.asarray(goal)#np.array([7, -10, 1.0]) # this is the goal
        self.goal_moving = False
        self.reached_goal = False
        
        self.target_velocity = 1.0
        self.RHP_time = 0.0 # receding horizon planning time, keeps track of the time for the trajectory
        self.tracking_in_global_time = 0
        self.Pglobal = np.asarray(ip)
        self.Vglobal = np.array([0,0,0])
        self.Rglobal = np.array([[1,0,0], [0,1,0], [0,0,1]])

        self.start = time.time()
        self.no_of_segments = 5
        self.replanning_counter = 1

        self.v_eoe = np.array([0,0,0])
        self.a_eoe = np.array([0,0,0])
        self.j_eoe = np.array([0,0,0])
        self.directionvector = np.array([1,0,0])
        self.execution_time = 0.067 # send trajectory for execution for this time only unless stopping policy is revoked

        
        self.Rcg_vibase = np.array([[1.0, 0.0, 0.0, 0.12], [0.0, 1.0, 0.0, 0.04], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        self.Rcdoc_cdl = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        self.Rvibase_cl = np.array([[1, 0, 0, 0.0], [0, 1, 0, 0.015], [0, 0, 1, 0.0], [0, 0, 0, 1]])
        #self.goal = np.array([9.5,0,1])

        self.pcl_pub = rospy.Publisher('/'+name+no+'/d435/cloud_for_trajectory', PointCloud2, queue_size = 1, tcp_nodelay = True)
        self.pcl_pub2 = rospy.Publisher('/'+name+no+'/local_goal', PointCloud2, queue_size = 1, tcp_nodelay = True)
        
        self.path_pub = rospy.Publisher('/'+name+no+'/path', Path, queue_size=1, tcp_nodelay=True)
        self.uav_traj_pub = rospy.Publisher('/'+name+no+'/polynomial_trajectory', MultiDOFJointTrajectory, queue_size = 1, tcp_nodelay = True)
        self.traj_polycoeff = rospy.Publisher('/'+name+no+'/polynomial_coefficients', PolynomialCoefficients, queue_size = 1, tcp_nodelay = True, latch = True)

        
        
        rospy.Subscriber('/'+name+no+'/d435/depth_registered/voxel_points', PointCloud2, self.planning_callback, tcp_nodelay = True)
        rospy.Subscriber('/'+name+no+'/modifiedodom', Odometry, self.odom_callback, tcp_nodelay = True)
        #rospy.Subscriber('/firefly1/polynomial_coefficients', PolynomialCoefficients, self.get_back_traj, tcp_nodelay = True)

    def get_back_traj(self, data): 
        
        if data.trajectory_mode == 'planning_in_camera_fov': 
            self.segment_times = data.segment_end_times
            self.planning_horizon = data.number_of_segments
            self.sensor_update_horizon = data.sensor_update_segment 
            #self.trajectory_start_time = data.planning_start_time
            #print 'horizons are', self.planning_horizon, self.sensor_update_horizon
        #self.replanning_started += 1

        

    def odom_callback(self, data):
        # get positions
        self.odom_time = data.header.stamp.to_sec()
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = self.q.rotation_matrix
        V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.Vglobal = np.dot(self.Rglobal, V)
        #rpy = self.quaternion_to_euler(self.q[1], self.q[2], self.q[3], self.q[0])
        #self.yaw = rpy[0]
        vector = np.array([self.goal[0]-self.Pglobal[0], self.goal[1]-self.Pglobal[1], self.goal[2]-self.Pglobal[2]])
        self.directionvector = vector/np.linalg.norm(vector)

        
        if self.planning_counter == 0: 
            self.initial_position = self.Pglobal
            

        
    def dist(self, p1,p2):
        return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)



    def generate_regular_pyramid_grid2(self, r): 

        xmax = r*np.tan(self.cone_angle_x/2)
        ymax = r*np.tan(self.cone_angle_y/2)

        NumberOfPointsX = int(2*xmax/self.resolution)
        NumberOfPointsY = int(2*ymax/self.resolution)
        x = np.linspace(-xmax, xmax, NumberOfPointsX); y = np.linspace(-ymax, ymax, NumberOfPointsY)

        xx, yy = np.meshgrid(x, y)
        p = np.stack((xx.ravel(), yy.ravel()), axis = -1)
        p = np.column_stack((p, np.ones(len(p))*r))

        return p
        

    def generate_regular_pyramid_grid(self, r): 

        ymax = r*np.tan(self.cone_angle_x/2); zmax = r*np.tan(self.cone_angle_y/2)
        #self.resolution = 0.1
        NumberOfPointsY = int(2*ymax/self.resolution)
        NumberOfPointsZ = int(2*zmax/self.resolution)
        y = np.linspace(-ymax, ymax, NumberOfPointsY); z = np.linspace(-zmax, zmax, NumberOfPointsZ)

        yy, zz = np.meshgrid(y, z)
        p = np.stack((yy.ravel(), zz.ravel()), axis = -1)
        p = np.column_stack((np.ones(len(p))*r, p))

        return p
        

    def check_collision_get_cost(self, ps, p, occupied_points, localgoal): 

        kdt_occ_points = cKDTree(occupied_points)

        n = 10; remove = []; Wcollision = []; Wdist2goal = []
 
        for i in range(len(p)):            
            points_on_line = np.linspace(ps, p[i], n+1, axis=0)
            kdt_p = cKDTree(points_on_line)
            indices = kdt_p.query_ball_tree(kdt_occ_points, self.safety_radius)
            if any(indices): 
                remove.append(i)
            else: #if any(indices) == False: 
                
                Wdist2goal.append(np.linalg.norm(localgoal-p[i]))
                distance = kdt_occ_points.query(points_on_line, k = 2)
                Wcollision.append(sum(distance[0].ravel()))
                
                
        collision_free_lines = np.delete(p, remove, 0)
        
        return collision_free_lines, Wdist2goal, Wcollision
 

    def transform_cloud(self, ps, occupied_points): 
        
        occ = np.zeros((occupied_points.shape[0],3))
        occ[:,0] = occupied_points['x']
        occ[:,1] = occupied_points['y']
        occ[:,2] = occupied_points['z']


        p1 = np.dot(self.Rcdoc_cdl[0:3,0:3], occ.T).T
        p2 = self.Rvibase_cl[0:3, 3:4].T + p1
        p3 = self.Rcg_vibase[0:3, 3:4].T + np.dot(self.Rcg_vibase[0:3, 0:3].T, p2.T).T 
        occ_p = self.Pglobal[np.newaxis] + np.dot(self.Rglobal, p3.T).T       
        occ_p = np.delete(occ_p, np.argwhere(occ_p[:,2] <=0.2).ravel(), axis=0)

        if len(occ_p) != 0:            
            return occ_p
        else: 
            return np.array([])
                

    def get_velocity(self, currenttime, d): 
        r = 1.0
        k1 = 0.5
        k2 = 0.2
        f1  = erf(k1*(d-r)); f2 = erf(k2*currenttime)        
        vel = self.uav_velocity*f1*f2 

        return vel, f1, f2
           
    def plan_in_camera_fov(self, pc_data):
        a = time.time()
       
        self.RHP_time = pc_data.header.stamp.to_sec()-self.hover_start_time
        distance_to_goal = np.linalg.norm(self.goal-self.Pglobal)
        if self.replanning_counter == 1: 
            ps = self.Pglobal
            direction = self.initial_orientation
            self.replanning_start_time = time.time()
            currenttime = 0.1
        else: 
            ps = self.Pglobal#self.p_eoe
            vector = self.goal-self.Pglobal
            direction = vector/np.linalg.norm(vector)
            currenttime = time.time()-self.replanning_start_time
            
        if self.goal_moving == False and distance_to_goal > self.sensor_range:        
            goal_vector = self.goal-ps
            self.lgoal = ps + goal_vector*self.sensor_range/np.linalg.norm(goal_vector)        
    
            r = min(distance_to_goal, self.sensor_range)    
            end_points =  self.generate_regular_pyramid_grid2(r)
            p11 = np.dot(self.Rcdoc_cdl[0:3,0:3], end_points.T).T
            p22 = self.Rvibase_cl[0:3, 3:4].T + p11
            p33 = self.Rcg_vibase[0:3, 3:4].T + np.dot(np.linalg.inv(self.Rcg_vibase[0:3, 0:3]), p22.T).T 
            end_points = ps[np.newaxis] + np.dot(self.Rglobal, p33.T).T
            
    
            occupied_points = ros_numpy.numpify(pc_data) 
            
            
            if len(occupied_points) != 0: 
                transformed_cloud = self.transform_cloud(ps, occupied_points)
                if self.replanning_counter == 100:
                    for i in transformed_cloud: 
                        f3 = open(self.path+'occ_points.txt', 'a')
                        f3.write('%s, %s\n' % (pc_data.header.stamp.to_sec(),i))
                if len(transformed_cloud) == 0: 
    
                    final_points = end_points
                    Wcollision = [1]*len(final_points)
                    Wdist2goal = np.linalg.norm(self.lgoal-final_points, axis = 1)
                    Wdist2goal = map(lambda i: i*1.0/max(Wdist2goal), Wdist2goal)
                else: 
                    final_points, Wdist2goal, Wcollision = self.check_collision_get_cost(ps, end_points, transformed_cloud, self.lgoal)
                    Wcollision = map(lambda i: i*1.0/max(Wcollision), Wcollision)
                    Wdist2goal = map(lambda i: i*1.0/max(Wdist2goal), Wdist2goal)
            else: 
                final_points = end_points
                Wcollision = [1]*len(final_points)
                Wdist2goal = np.linalg.norm(self.lgoal-final_points, axis = 1)
                Wdist2goal = map(lambda i: i*1.0/max(Wdist2goal), Wdist2goal)            
    
            #print 'length of end points and occupied points' , len(Wdist2goal), len(Wcollision), len(final_points)
            Wtotal = map(lambda i, j: 3*i+1.0/j, Wdist2goal, Wcollision)
            #print 'wtotal len', len(Wtotal)
            local_goal = final_points[np.argmin(Wtotal)]
      
            n = self.no_of_segments
            #self.N = 8
            path =  zip(np.linspace(ps[0], local_goal[0], n+1), np.linspace(ps[1], local_goal[1], n+1), np.linspace(ps[2], local_goal[2], n+1))
            
            vel, f1, f2 = self.get_velocity(currenttime, distance_to_goal)
    
            f = open(self.path+'velocities.txt', 'a')
            f.write('%s, %s, %s, %s, %s\n' % (currenttime, distance_to_goal, f1, f2, vel))
            
            T, _p1, _p2, _p3 = Minimum_jerk_trajetory(self.replanning_counter, vel, path, self.v_eoe, self.a_eoe, self.j_eoe).construct_trajectory()      
    
            _pp1 = [_p1[i:i + self.N] for i in range(0, len(_p1), self.N)]
            [i.reverse() for i in _pp1]
        
            _pp2 = [_p2[i:i + self.N] for i in range(0, len(_p2), self.N)]
            [i.reverse() for i in _pp2]
        
            _pp3 = [_p3[i:i + self.N] for i in range(0, len(_p3), self.N)]
            [i.reverse() for i in _pp3]
    
        
            #t = map(lambda i: np.linspace(T[i], T[i+1], 201), range(1)) 
            tt = self.execution_time
            index = len(np.where(np.array(T)<tt)[0])-1
            
            
            
            xx = np.poly1d(_pp1[index]); yy = np.poly1d(_pp2[index]); zz = np.poly1d(_pp3[index])
            vx = np.polyder(xx, 1); vy = np.polyder(yy, 1); vz = np.polyder(zz, 1)
            accx = np.polyder(xx, 2); accy = np.polyder(yy, 2); accz = np.polyder(zz, 2)
            jx = np.polyder(xx, 3); jy = np.polyder(yy, 3); jz = np.polyder(zz, 3)
            #index = np.argmin(np.abs(np.array(t[0])-tt))
    
            self.p_eoe = np.array([xx(tt), yy(tt), zz(tt)])
            self.v_eoe = np.array([vx(tt), vy(tt), vz(tt)])
            self.a_eoe = np.array([accx(tt), accy(tt), accz(tt)])
            self.j_eoe = np.array([jx(tt), jy(tt), jz(tt)])
            
    
    
    
            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp = pc_data.header.stamp
            header.frame_id = 'map'
            msg.header = header
            msg.polynomial_order = self.N-1
            for i in range(len(_p1)): 
                msg.poly_x.append(_p1[i]); msg.poly_y.append(_p2[i]); msg.poly_z.append(_p3[i])
            msg.number_of_segments = n
            #msg.sensor_update_segment = N_sensor_update
            msg.planning_start_time = str(self.odom_time)
            for j in T:
                msg.segment_end_times.append(j)
            msg.desired_direction.x = direction[0]
            msg.desired_direction.y = direction[1]; msg.desired_direction.z = direction[2]
    
            msg.trajectory_mode = 'planning_in_camera_fov'
            msg.execution_time_horizon = str(self.execution_time)
            self.previous_msg = msg
             
            self.traj_polycoeff.publish(msg)
            self.trajectory_endtime = T[-1]
            #print '1', time.time()-a
            """
            #****************** only for rviz visualization will be deleted eventually **************************************************
    
            #eoe_msg = Odometry()
            #eoe_msg.pose.pose.position.x = self.p_eoe[0]
            #eoe_msg.pose.pose.position.y = self.p_eoe[1]
            #eoe_msg.pose.pose.position.z = self.p_eoe[2]
            #eoe_msg.header.stamp = rospy.Time.now()
            #eoe_msg.header.frame_id = 'map'
            #self.eoe_odom.publish(eoe_msg)
    
    
            t = np.linspace(T[:n], T[1:n+1], 20, axis=1)
            
            xx = map(lambda i: np.poly1d(_pp1[i]), range(n))
            yy = map(lambda i: np.poly1d(_pp2[i]), range(n))
            zz = map(lambda i: np.poly1d(_pp3[i]), range(n))
            
            vx = map(lambda i: np.polyder(xx[i], 1), range(n))
            vy = map(lambda i: np.polyder(yy[i], 1), range(n))
            vz = map(lambda i: np.polyder(zz[i], 1), range(n))
            
            accx = map(lambda i: np.polyder(xx[i], 2), range(n))
            accy = map(lambda i: np.polyder(yy[i], 2), range(n))
            accz = map(lambda i: np.polyder(zz[i], 2), range(n))
    
            jx = map(lambda i: np.polyder(xx[i], 3), range(n))
            jy = map(lambda i: np.polyder(yy[i], 3), range(n))
            jz = map(lambda i: np.polyder(zz[i], 3), range(n))
            
            
                    
            #print '12', time.time()-a
            #f = open(self.path+'end_points.txt', 'a')
            #f.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n' %(time.time()-a, vel, f1, f2, local_goal, tt, T[1], currenttime, self.RHP_time, pc_data.header.stamp.to_sec(), self.hover_start_time)) 
                
            self.traj = MultiDOFJointTrajectory()
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            self.traj.header = header
            self.traj.joint_names = 'mav_cog' # testing for now
            
            self.pathh = Path()    
            self.pathh.header.stamp = rospy.Time.now()
            self.pathh.header.frame_id = 'map'
    
            start_time =self.odom_time
            for i in range(n): # "changed to no_of_segments_to_track" instead of "no_of_segments" 
                for j in t[i]:
                    pose = PoseStamped()
                    pose.header = pc_data.header 
                    ttt = j + start_time    
                    xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                    vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                    axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
    
                    vector = np.array([self.goal[0]-self.Pglobal[0], self.goal[1]-self.Pglobal[1], self.goal[2]-self.Pglobal[2]])
                    #vector = np.array([self.Vglobal[0], self.Vglobal[1], self.Vglobal[2]])
                    direction = vector/np.linalg.norm(vector)
                    
                    transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                    velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                    accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(direction[0],direction[1],direction[2]))
                    #accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(1,0,0))
                    
                    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(ttt))
                    self.traj.points.append(point)
                    pose.pose.position.x = xdes; pose.pose.position.y = ydes; pose.pose.position.z = zdes
                    pose.pose.orientation.x = 0; pose.pose.orientation.y = 0; pose.pose.orientation.z = 0; pose.pose.orientation.w = 1
                    self.pathh.poses.append(pose)
    
            self.uav_traj_pub.publish(self.traj)
            self.path_pub.publish(self.pathh)
            
            
            #publishing 
           
    
            f1 = open(self.path+'goals.txt', 'a')
            f1.write('%s, %s, %s, %s, %s\n' % (time.time()-a, index, T[1], self.Pglobal, ps))
            
            # publish generated pointcloud
            header = std_msgs.msg.Header()
            header.stamp = pc_data.header.stamp
            header.frame_id = 'map'#, 'firefly1/vi_sensor/camera_depth_optical_center_link'
            p = pc2.create_cloud_xyz32(header, transformed_cloud)
            self.pcl_pub.publish(p) 
            
            # publish local_goal
            header = std_msgs.msg.Header()
            header.stamp = pc_data.header.stamp
            header.frame_id = 'map'
            p = pc2.create_cloud_xyz32(header, final_points)
            self.pcl_pub2.publish(p) 
            #****************** will be deleted eventually **********************************************************************************
            """
            print '2', time.time()-a
            
        
        else:  
            if RHPendcounter == 0: 
                self.goal_in_fov = self.RHP_time
            self.previous_msg.execution_time_horizon = str(self.trajectory_endtime)
            self.traj_polycoeff.publish(self.previous_msg)
            self.RHPendcounter += 1
            if self.RHP_time > self.goal_in_fov+self.trajectory_endtime+10:
                self.reached_goal = True 

        
        print time.time()-a
        self.replanning_counter += 1
        
        

    def hover_up(self, data): 
        """give commands to hover at 1m above the starting position"""

        if self.hover_counter == 0: 
            self.hover_start_time = data.header.stamp.to_sec()
        self.RHP_time = data.header.stamp.to_sec()-self.hover_start_time
        if self.hover_counter == 0:
            path = [[self.initial_position[0], self.initial_position[1], self.initial_position[2]], [self.initial_position[0], self.initial_position[1], self.hovering_height]]

            vel = np.array([0,0,0]); acc = np.array([0,0,0]); jerk = np.array([0,0,0])
            counter = 0; velocity = self.vel_hl

            T, p1, p2, p3 = Minimum_jerk_trajetory(counter, velocity, path, vel, acc, jerk).construct_trajectory() 

            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp =  data.header.stamp#rospy.Time.now()
            header.frame_id = 'map'
            msg.header = header
            msg.polynomial_order = 7
            for i in range(len(p1)): 
                msg.poly_x.append(p1[i]); msg.poly_y.append(p2[i]); msg.poly_z.append(p3[i])
            msg.number_of_segments = 1
            msg.planning_start_time =  str(self.hover_start_time)
            for j in T: 
                msg.segment_end_times.append(j)
            msg.desired_direction.x = self.initial_orientation[0]; msg.desired_direction.y = self.initial_orientation[1]
            msg.desired_direction.z = self.initial_orientation[2]
            msg.execution_time_horizon = str(0) # dummy value
            msg.trajectory_mode = 'hover'
            self.traj_polycoeff.publish(msg)

        self.hover_counter += 1


    def land(self, data): 
        """give commands to hover at 1m above the starting position"""

        if self.land_counter == 0: 
            self.land_start_time = data.header.stamp.to_sec()
            self.land_start = [self.Pglobal[0], self.Pglobal[1], self.Pglobal[2]]
        if self.land_counter == 0:
            path = [[self.land_start[0], self.land_start[1], self.land_start[2]], [self.land_start[0], self.land_start[1], 0.1]]
            vel = np.array([0,0,0]); acc = np.array([0,0,0]); jerk = np.array([0,0,0])
            counter = 0; velocity = self.vel_hl
        
            T, p1, p2, p3 = Minimum_jerk_trajetory(counter, velocity, path, vel, acc, jerk).construct_trajectory()      
            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp =  data.header.stamp#rospy.Time.now()
            header.frame_id = 'map'
            msg.header = header
            msg.polynomial_order = 7
            for i in range(len(p1)): 
                msg.poly_x.append(p1[i]); msg.poly_y.append(p2[i]); msg.poly_z.append(p3[i])
            msg.number_of_segments = 1
            msg.planning_start_time =  str(self.land_start_time)
            for j in T: 
                msg.segment_end_times.append(j)
            msg.desired_direction.x = self.initial_orientation[0]; msg.desired_direction.y = self.initial_orientation[1]
            msg.desired_direction.z = self.initial_orientation[2]
            msg.execution_time_horizon = str(0) # dummy value
            msg.trajectory_mode = 'land'
            self.traj_polycoeff.publish(msg)

        self.land_counter += 1  

    def planning_callback(self, data): 
        """main motion planner, plan different trajectories for different stages"""
        #self.trajectory_in_camera_fov(data)

        #start generating the trajectory
        #if self.RHP_time <= self.initial_hover_time and self.reached_goal == False: 
        #    self.hover_up(data)
        #elif self.RHP_time > self.initial_hover_time and self.reached_goal == False:
        #    self.plan_in_camera_fov(data)
        #elif self.RHP_time > self.initial_hover_time and self.reached_goal == True: 
        #    self.land(data)   

        #only hover up and down 
        if self.RHP_time <= self.initial_hover_time and self.reached_goal == False: 
            self.hover_up(data)
        elif self.RHP_time > self.initial_hover_time: 
            self.land(data) 

        self.planning_counter += 1
            
# if python says run, then we should run
if __name__ == '__main__': 
    rospy.init_node('trajectory_in_camera_fov', anonymous=True, log_level=rospy.DEBUG)
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    initial_position = rospy.get_param('initial_position')
    initial_orientation = rospy.get_param('initial_orientation')
    res = rospy.get_param('visibility_graph_resolution')
    radius = rospy.get_param('uav_radius')
    sr = rospy.get_param('sensor_range')    
    fov_x = rospy.get_param('camera_fov_x')
    fov_y = rospy.get_param('camera_fov_y')
    h = rospy.get_param('initial_hovering_height')
    ht = rospy.get_param('initial_hovering_time')
    vel = rospy.get_param('uav_velocity')
    goal = rospy.get_param('goal_point')
    order = rospy.get_param('polynomial_order')
    vel_hl = rospy.get_param('velocity_during_hover_land')

    rospy.init_node('trajectory_in_camera_fov', anonymous=True, log_level=rospy.DEBUG)
    
    f = test(name, number, initial_position, initial_orientation, res, radius, sr, fov_x, fov_y, h, vel, goal, ht, order, vel_hl)
    
    #f.calculate()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
       pass


