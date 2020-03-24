#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
new script for trajectory generation in camera FOV from scratch
"""
import time, itertools, math
import numpy as np
from math import sqrt

from pyquaternion import Quaternion as Qt
from scipy.spatial import cKDTree
from scipy.special import erf
from polynomial_trajectory_generation_newlogic import Minimum_jerk_trajetory
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Point, Quaternion, PoseStamped
from geometric_controller.msg import PolynomialCoefficients



class RHP(object): 
    def __init__(self, name, number, ip, io, res, radius, sr, fov_x, fov_y, h, vel, goal, ht, order, vel_hl, RN, do_replanning):
        """initialization"""
        self.path = '/home/pelican/data_collection/rhp_logs/'
        open(self.path+'goals_RN{}.txt'.format(RN), 'w').close()
        open(self.path+'velocities_RN{}.txt'.format(RN), 'w').close()
        open(self.path+'yaw_values_RN{}.txt'.format(RN), 'w').close()
        open(self.path+'time_taken_by_script_RN{}.txt'.format(RN), 'w').close()
        open('len_of_transformed_cloud_RN{}.txt'.format(RN), 'w').close()
        self.do_replanning = do_replanning
        self.RN = RN
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
        self.execution_time = 0.067 # send trajectory for execution for this time only unless stopping policy is revoked
        self.stopping_now = False
        self.emergency_stop = False

        
        self.Rcg_vibase = np.array([[1.0, 0.0, 0.0, 0.12], [0.0, 1.0, 0.0, 0.02], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        self.Rcdoc_cdl = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        self.Rvibase_cl = np.array([[1, 0, 0, 0.0], [0, 1, 0, 0.015], [0, 0, 1, 0.0], [0, 0, 0, 1]])


        #self.pcl_pub = rospy.Publisher('/'+name+no+'/d435/cloud_for_trajectory', PointCloud2, queue_size = 1, tcp_nodelay = True)
        #self.pcl_pub2 = rospy.Publisher('/'+name+no+'/local_goal', PointCloud2, queue_size = 1, tcp_nodelay = True)
        
        #self.path_pub = rospy.Publisher('/'+name+no+'/path', Path, queue_size=1, tcp_nodelay=True)
        self.uav_traj_pub = rospy.Publisher('/'+name+no+'/polynomial_trajectory', MultiDOFJointTrajectory, queue_size = 10, tcp_nodelay = True, latch=True)


        rospy.Subscriber('/'+name+no+'/d435/depth_registered/voxel_points', PointCloud2, self.planning_callback, tcp_nodelay = True)
        rospy.Subscriber('/'+name+no+'/modifiedodom', Odometry, self.odom_callback, tcp_nodelay = True)

    def odom_callback(self, data):
        # get positions
        self.odom_time = data.header.stamp.to_sec()
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = self.q.rotation_matrix
        V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.Vglobal = np.dot(self.Rglobal, V)

        if self.planning_counter == 0: 
            self.initial_position = self.Pglobal

    def generate_regular_pyramid_grid(self, r): 
 
        rr = np.linspace(r, 0.5*r, 3)
        pp = np.zeros((0,3))
        for i in rr: 
            xmax = i*np.tan(self.cone_angle_x/2)
            ymax = i*np.tan(self.cone_angle_y/2)
    
            NumberOfPointsX = int(2*xmax/self.resolution)
            NumberOfPointsY = int(2*ymax/self.resolution)
            x = np.linspace(-xmax + i*self.resolution/2, xmax - i*self.resolution/2, NumberOfPointsX)
            y = np.linspace(-ymax + i*self.resolution/2, ymax - i*self.resolution/2, NumberOfPointsY)
    
            xx, yy = np.meshgrid(x, y)
            p = np.stack((xx.ravel(), yy.ravel()), axis = -1)
            p = np.column_stack((p, np.ones(len(p))*i))
            pp = np.concatenate((pp, p), 0)
        return pp


    def check_collision_get_cost(self, ps, p, occupied_points, localgoal): 
        a = time.time()
        kdt_occ_points = cKDTree(occupied_points)
        n = 10; remove = []
        SR = 0.75 # second safety radius
	rr = self.safety_radius
        factor = (1 + SR**4)/SR**4
        
        points_on_lines = np.linspace(ps, p, n+1, axis = 1) 
        points_on_lines = np.vstack(points_on_lines)
        #kdt_p = cKDTree(points_on_lines)
        indices = np.array(kdt_occ_points.query_ball_point(points_on_lines, self.safety_radius, eps = 0.1))
        indices = indices.reshape(-1, n+1)
        
        for i in range(len(indices)): 
            if any(indices[i]): 
                remove.append(i)
 
        collision_free_lines = np.delete(p, remove, 0)

        
        points_on_lines = np.linspace(ps, collision_free_lines, n+1, axis = 1)
        points_on_lines = np.vstack(points_on_lines) #it flattens the array, np.flatten was not working probably becuase of dtype 

        distance = kdt_occ_points.query(points_on_lines, k = 1, eps = 0.1, n_jobs = -1)
        distance = np.min(distance[0].reshape(-1, n+1), axis = 1)

        #Wcollision = np.array(map(lambda i: factor* (i**2- SR**2)**2/ (1 + (i**2- SR**2)**2) if i < SR else 0, distance))
        Wcollision = np.array(map(lambda i: factor* ((i-rr)**2- SR**2)**2/ (1 + ((i-rr)**2- SR**2)**2) if i-rr < SR else 0, distance))

        Wdist2goal = np.linalg.norm(localgoal-collision_free_lines, axis = 1)
        Wdist2goal = Wdist2goal/max(Wdist2goal)
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
        #occ_p = np.delete(occ_p, np.argwhere(occ_p[:,2] <=0.3).ravel(), axis=0)

	return occ_p

                

    def get_velocity(self, currenttime, d, velocity): 
        r = 1.0
        k1 = 0.3
        k2 = 0.2
        f1  = erf(k1*(d-r)); f2 = erf(k2*currenttime)        
        vel = velocity*f1*f2 

        return vel, f1, f2
 
    def getyaw(self, y0, yr0, yT, yrT, t): 
        """return smooth yaw for the trajectory"""
        T = t[-1][-1]
        a = y0
        b = yr0
        c = 3 * (yT-y0) / T**2 - (2 * yr0 + yrT) / T
        d = (yr0 + yrT) / T**2 - 2 * (yT-y0) / T**3
        yaw = [map(lambda i: a + b*i + c*i**2 + d*i**3, j) for j in t]
        yawrate = [map(lambda i: b + 2*c*i + 3*d*i**2, j) for j in t]
        
        return yaw, yawrate

          
    def plan_in_camera_fov(self, pc_data):
        a = time.time()

        self.RHP_time = pc_data.header.stamp.to_sec()-self.hover_start_time
        distance_to_goal = np.linalg.norm(self.goal-self.Pglobal)
        
        if self.stopping_now == True:            
            if self.RHP_time > self.goal_in_fov + self.replanning_endtime + 5.0: 
                self.reached_goal = True
                
        if self.replanning_counter == 1: 
            yaw0 = 0.0; yawrate0 = 0
            ps = self.p_hover_end
            direction = self.initial_orientation
            self.replanning_start_time = time.time()
            currenttime = 0.1
        else: 
            yaw0 = self.yaw_eoe; yawrate0 = self.yawrate_eoe
            ps = self.p_eoe
            #vector = self.lgoal-self.Pglobal
            #direction = vector/np.linalg.norm(vector)
            currenttime = time.time()-self.replanning_start_time
            
        goal_vector = self.goal-ps
        self.lgoal = ps + goal_vector*self.sensor_range/np.linalg.norm(goal_vector)        

        r = min(distance_to_goal+0.5, self.sensor_range)  
        end_points =  self.generate_regular_pyramid_grid(r)
        
        p11 = np.dot(self.Rcdoc_cdl[0:3,0:3], end_points.T).T
        p22 = self.Rvibase_cl[0:3, 3:4].T + p11
        p33 = self.Rcg_vibase[0:3, 3:4].T + np.dot(self.Rcg_vibase[0:3, 0:3].T, p22.T).T 
        end_points = ps[np.newaxis] + np.dot(self.Rglobal, p33.T).T
        

        occupied_points = ros_numpy.numpify(pc_data) 


        if len(occupied_points) != 0: 
            transformed_cloud = self.transform_cloud(ps, occupied_points)
            bb = time.time()
            final_points, Wdist2goal, Wcollision = self.check_collision_get_cost(ps, end_points, transformed_cloud, self.lgoal)
            Wtotal = Wdist2goal + 2.0*Wcollision
            Wtotal = Wtotal/max(Wtotal)
        else: 
            
            final_points = end_points
            Wdist2goal = np.linalg.norm(self.lgoal-final_points, axis = 1)
            Wtotal = Wdist2goal/max(Wdist2goal)

        if np.all(np.linalg.norm(final_points-ps, axis = 1) < 2.8) and self.stopping_now == False:  
            self.emergency_stop = True        
        print 'emergancy  stop is:', self.emergency_stop

        local_goal = final_points[np.argmin(Wtotal)]
        
  
        n = self.no_of_segments
        path =  zip(np.linspace(ps[0], local_goal[0], n+1), np.linspace(ps[1], local_goal[1], n+1), np.linspace(ps[2], local_goal[2], n+1))
        
        self.traj = MultiDOFJointTrajectory()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        self.traj.header = header
        self.traj.joint_names = 'mav_cog' 
        
        if self.replanning_counter != 1: 
            direction = (local_goal-ps)/np.linalg.norm(local_goal-ps)

        yawT = np.arctan2(direction[1], direction[0]) # get yaw
        #yawT = yawT if yawT >=0 else 2*np.pi+yawT # covert it into [0, 2*pi]
        yawrateT  = 0 
        
        if distance_to_goal < self.sensor_range and self.RHPendcounter == 0  or self.emergency_stop == True:
            path =  zip(np.linspace(ps[0], self.goal[0], n+1), np.linspace(ps[1], self.goal[1], n+1), np.linspace(ps[2], self.goal[2], n+1))
            vel, f1, f2 = self.get_velocity(currenttime, distance_to_goal, self.uav_velocity)
            
            f = open(self.path+'velocities_RN{}.txt'.format(self.RN), 'a')
            f.write('%s, %s, %s, %s, %s\n' % (currenttime, distance_to_goal, f1, f2, vel))
            
            T, _p1, _p2, _p3 = Minimum_jerk_trajetory(self.replanning_counter, vel, path, self.v_eoe, self.a_eoe, self.j_eoe).construct_trajectory()      
    
            _pp1 = [_p1[i:i + self.N] for i in range(0, len(_p1), self.N)]
            [i.reverse() for i in _pp1]
        
            _pp2 = [_p2[i:i + self.N] for i in range(0, len(_p2), self.N)]
            [i.reverse() for i in _pp2]
        
            _pp3 = [_p3[i:i + self.N] for i in range(0, len(_p3), self.N)]
            [i.reverse() for i in _pp3]

            t = np.linspace(T[:n], T[1:n+1], 71, axis=1)
            yaw, yawrate = self.getyaw(yaw0, yawrate0, yawT, yawrateT, t)
            self.yaw_end = yaw[-1][-1]

            xx = map(lambda i: np.poly1d(_pp1[i]), range(n))
            yy = map(lambda i: np.poly1d(_pp2[i]), range(n))
            zz = map(lambda i: np.poly1d(_pp3[i]), range(n))
            
            vx = map(lambda i: np.polyder(xx[i], 1), range(n))
            vy = map(lambda i: np.polyder(yy[i], 1), range(n))
            vz = map(lambda i: np.polyder(zz[i], 1), range(n))
            
            accx = map(lambda i: np.polyder(xx[i], 2), range(n))
            accy = map(lambda i: np.polyder(yy[i], 2), range(n))
            accz = map(lambda i: np.polyder(zz[i], 2), range(n))
    
            #jx = map(lambda i: np.polyder(xx[i], 3), range(n))
            #jy = map(lambda i: np.polyder(yy[i], 3), range(n))
            #jz = map(lambda i: np.polyder(zz[i], 3), range(n))


            a1 = time.time()-a
            start_time =self.odom_time
            for i in range(n): 
                k = 0
                for j in t[i]:
                    ttt = j + start_time    
                    xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                    vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                    axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
    
                    yaw_now = yaw[i][k]
                    desd = np.array([np.cos(yaw_now), np.sin(yaw_now), 0])
                    #desd = vector/np.linalg.norm(vector)
                    
                    transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                    velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                    accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(desd[0],desd[1],desd[2]))
                    #accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(1,0,0))
                    
                    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(ttt))
                    self.traj.points.append(point)
                    k += 1

            #self.uav_traj_pub.publish(self.traj)


            #if self.RHPendcounter == 0: 
            #    self.goal_in_fov = self.RHP_time
            #    self.replanning_endtime =  T[-1]
            #    self.stopping_now = True
            #self.RHPendcounter +=1   

            if self.RHPendcounter == 0 and self.emergency_stop == False: 
                self.goal_in_fov = self.RHP_time
                self.replanning_endtime =  T[-1]                
                self.stopping_now = True 
                self.RHPendcounter +=1  
                self.uav_traj_pub.publish(self.traj)

            elif self.RHPendcounter == 0 and self.emergency_stop == True:
                self.goal_in_fov = self.RHP_time
                self.replanning_endtime =  T[-1]                
                self.stopping_now = True 
                self.RHPendcounter +=1
                self.uav_traj_pub.publish(self.traj)


        elif distance_to_goal >= self.sensor_range and self.stopping_now == False and self.emergency_stop == False:             
        
            vel, f1, f2 = self.get_velocity(currenttime, distance_to_goal, self.uav_velocity)
            
            f = open(self.path+'velocities_RN{}.txt'.format(self.RN), 'a')
            f.write('%s, %s, %s, %s, %s\n' % (currenttime, distance_to_goal, f1, f2, vel))
            
            T, _p1, _p2, _p3 = Minimum_jerk_trajetory(self.replanning_counter, vel, path, self.v_eoe, self.a_eoe, self.j_eoe).construct_trajectory()      
    
            _pp1 = [_p1[i:i + self.N] for i in range(0, len(_p1), self.N)]
            [i.reverse() for i in _pp1]
        
            _pp2 = [_p2[i:i + self.N] for i in range(0, len(_p2), self.N)]
            [i.reverse() for i in _pp2]
        
            _pp3 = [_p3[i:i + self.N] for i in range(0, len(_p3), self.N)]
            [i.reverse() for i in _pp3]
           
            t = np.linspace(T[:n], T[1:n+1], 51, axis=1)
            yaw, yawrate = self.getyaw(yaw0, yawrate0, yawT, yawrateT, t)
            
            tt = self.execution_time
            index = len(np.where(np.array(T)<tt)[0])-1 # find where tt is in T
            subindex = len(np.where(t[index]<tt)[0])-1 # find where tt is T
            
            # remove this and take self.p_eoe below map
            #xx = np.poly1d(_pp1[index]); yy = np.poly1d(_pp2[index]); zz = np.poly1d(_pp3[index])
            #vx = np.polyder(xx, 1); vy = np.polyder(yy, 1); vz = np.polyder(zz, 1)
            #accx = np.polyder(xx, 2); accy = np.polyder(yy, 2); accz = np.polyder(zz, 2)
            #jx = np.polyder(xx, 3); jy = np.polyder(yy, 3); jz = np.polyder(zz, 3)

    


            xx = map(lambda i: np.poly1d(_pp1[i]), range(index+1))
            yy = map(lambda i: np.poly1d(_pp2[i]), range(index+1))
            zz = map(lambda i: np.poly1d(_pp3[i]), range(index+1))
            
            vx = map(lambda i: np.polyder(xx[i], 1), range(index+1))
            vy = map(lambda i: np.polyder(yy[i], 1), range(index+1))
            vz = map(lambda i: np.polyder(zz[i], 1), range(index+1))
            
            accx = map(lambda i: np.polyder(xx[i], 2), range(index+1))
            accy = map(lambda i: np.polyder(yy[i], 2), range(index+1))
            accz = map(lambda i: np.polyder(zz[i], 2), range(index+1))
    
            jx = map(lambda i: np.polyder(xx[i], 3), range(index+1))
            jy = map(lambda i: np.polyder(yy[i], 3), range(index+1))
            jz = map(lambda i: np.polyder(zz[i], 3), range(index+1))

            self.p_eoe = np.array([xx[index](tt), yy[index](tt), zz[index](tt)])
            self.v_eoe = np.array([vx[index](tt), vy[index](tt), vz[index](tt)])
            self.a_eoe = np.array([accx[index](tt), accy[index](tt), accz[index](tt)])
            self.j_eoe = np.array([jx[index](tt), jy[index](tt), jz[index](tt)])
            self.yaw_eoe = yaw[index][subindex]; self.yawrate_eoe = yawrate[index][subindex]
            
            f2 = open(self.path + 'yaw_values_RN{}.txt'.format(RN),'a')
            f2.write('%s, %s\n' %(self.yaw_eoe, self.yawrate_eoe))

            a1 = time.time()-a
    
            start_time =self.odom_time 
            for i in range(index+1): 
                k = 0
                for j in t[i][:subindex+1]:
                    ttt = j + start_time    
                    xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                    vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                    axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
                    
                    yaw_now = yaw[i][k]
                    desd = [np.cos(yaw_now), np.sin(yaw_now), 0]
                    #desd = vector/np.linalg.norm(vector)
            
                    transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                    velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                    accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(desd[0],desd[1],desd[2]))
                    
                    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(ttt))

                    self.traj.points.append(point)
                    k += 1

            self.uav_traj_pub.publish(self.traj)
        

        rospy.loginfo('Replanning execution time:%s' % (time.time()-a))
        f3 = open(self.path+'time_taken_by_script_RN{}.txt'.format(RN), 'a')
        f3.write('%s, %s, %s, %s, %s\n' % (time.time()-a, self.Pglobal, ps, self.lgoal, local_goal))

        self.replanning_counter += 1
        

    def hover_up(self, data): 
        """give commands to hover at 1m above the starting position"""

        if self.hover_counter == 0: 
            self.hover_start_time = data.header.stamp.to_sec()
        self.RHP_time = data.header.stamp.to_sec()-self.hover_start_time
        if self.hover_counter == 0:

            path = [[self.initial_position[0], self.initial_position[1], self.initial_position[2]], [self.initial_position[0]+0.5, self.initial_position[1], self.hovering_height]]

            vel = np.array([0,0,0]); acc = np.array([0,0,0]); jerk = np.array([0,0,0])
            planning_counter = 0; velocity = self.vel_hl
    
            T, _p1, _p2, _p3 = Minimum_jerk_trajetory(planning_counter, velocity, path, vel, acc, jerk).construct_trajectory()      

            _pp1 = [_p1[i:i + self.N] for i in range(0, len(_p1), self.N)]
            [i.reverse() for i in _pp1]
        
            _pp2 = [_p2[i:i + self.N] for i in range(0, len(_p2), self.N)]
            [i.reverse() for i in _pp2]
        
            _pp3 = [_p3[i:i + self.N] for i in range(0, len(_p3), self.N)]
            [i.reverse() for i in _pp3]


            self.traj = MultiDOFJointTrajectory()
            header = std_msgs.msg.Header()
            header.stamp = data.header.stamp
            header.frame_id = 'map'
            self.traj.header = header
            self.traj.joint_names = 'mav_cog' # testing for now

            t = np.linspace(T[:1], T[1:2], 101, axis=1)


            xx = map(lambda i: np.poly1d(_pp1[i]), range(1))
            yy = map(lambda i: np.poly1d(_pp2[i]), range(1))
            zz = map(lambda i: np.poly1d(_pp3[i]), range(1))
            
            vx = map(lambda i: np.polyder(xx[i], 1), range(1))
            vy = map(lambda i: np.polyder(yy[i], 1), range(1))
            vz = map(lambda i: np.polyder(zz[i], 1), range(1))
            
            accx = map(lambda i: np.polyder(xx[i], 2), range(1))
            accy = map(lambda i: np.polyder(yy[i], 2), range(1))
            accz = map(lambda i: np.polyder(zz[i], 2), range(1))
            self.p_hover_end = np.array([xx[-1](t[-1][-1]), yy[-1](t[-1][-1]), zz[-1](t[-1][-1])])


            start_time = data.header.stamp.to_sec()
            direction = [self.initial_orientation[0], self.initial_orientation[1], self.initial_orientation[2]]
            for i in range(1): 
                for j in t[i]:
                    ttt = j + start_time    
                    xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                    vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                    axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
                  
                    
                    transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                    velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                    accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(direction[0],direction[1],direction[2]))
                                 
                    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(ttt))

                    self.traj.points.append(point)

            self.uav_traj_pub.publish(self.traj)

 
        self.hover_counter += 1

    def land(self, data): 
        """give commands to hover at 1m above the starting position"""

        if self.land_counter == 0: 
            self.land_start_time = data.header.stamp.to_sec()
            self.land_start_time = data.header.stamp.to_sec()

            path = [[self.Pglobal[0], self.Pglobal[1], self.Pglobal[2]], [self.Pglobal[0], self.Pglobal[1], 0.05]]
            vel = np.array([0,0,0]); acc = np.array([0,0,0]); jerk = np.array([0,0,0])
            planning_counter = 0; velocity = self.vel_hl


            T, _p1, _p2, _p3 = Minimum_jerk_trajetory(planning_counter, velocity, path, vel, acc, jerk).construct_trajectory()      

            _pp1 = [_p1[i:i + self.N] for i in range(0, len(_p1), self.N)]
            [i.reverse() for i in _pp1]
        
            _pp2 = [_p2[i:i + self.N] for i in range(0, len(_p2), self.N)]
            [i.reverse() for i in _pp2]
        
            _pp3 = [_p3[i:i + self.N] for i in range(0, len(_p3), self.N)]
            [i.reverse() for i in _pp3]

            self.traj = MultiDOFJointTrajectory()
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            self.traj.header = header
            self.traj.joint_names = 'mav_cog' # testing for now


            t = np.linspace(T[:1], T[1:2], 101, axis=1)

            xx = map(lambda i: np.poly1d(_pp1[i]), range(1))
            yy = map(lambda i: np.poly1d(_pp2[i]), range(1))
            zz = map(lambda i: np.poly1d(_pp3[i]), range(1))
            
            vx = map(lambda i: np.polyder(xx[i], 1), range(1))
            vy = map(lambda i: np.polyder(yy[i], 1), range(1))
            vz = map(lambda i: np.polyder(zz[i], 1), range(1))
            
            accx = map(lambda i: np.polyder(xx[i], 2), range(1))
            accy = map(lambda i: np.polyder(yy[i], 2), range(1))
            accz = map(lambda i: np.polyder(zz[i], 2), range(1))


            
            start_time = data.header.stamp.to_sec()
            currentyaw = self.q.angle
            direction = [np.cos(currentyaw), np.sin(currentyaw), 0]
            for i in range(1): # "changed to no_of_segments_to_track" instead of "no_of_segments" 
                for j in t[i]:
                    ttt = j + start_time    
                    xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                    vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                    axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
                    
                    transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                    velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                    accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(direction[0],direction[1],direction[2]))
                    
                    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(ttt))
                    self.traj.points.append(point)

            self.uav_traj_pub.publish(self.traj)
            
        self.land_counter += 1   

    def planning_callback(self, data): 
        """main motion planner, plan different trajectories for different stages"""
        if self.do_replanning == True: 
            if self.RHP_time <= self.initial_hover_time and self.reached_goal == False: 
                rospy.loginfo('Hovering Up.')
                self.hover_up(data)
            elif self.RHP_time > self.initial_hover_time and self.reached_goal == False:
                #rospy.loginfo('Replanning Phase')
                self.plan_in_camera_fov(data)
            elif self.RHP_time > self.initial_hover_time and self.reached_goal == True:
                rospy.loginfo('Climbing Down.') 
                self.land(data)  
        else:  
            if self.RHP_time <= self.initial_hover_time and self.reached_goal == False: 
                rospy.loginfo('Hovering Up.')
                self.hover_up(data)
            elif self.RHP_time > self.initial_hover_time: 
                rospy.loginfo('Climbing Down.')
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
    RN = rospy.get_param('run_number')
    do_replanning = rospy.get_param('do_replanning')

    rospy.init_node('trajectory_in_camera_fov', anonymous=True, log_level=rospy.DEBUG)
    
    f = RHP(name, number, initial_position, initial_orientation, res, radius, sr, fov_x, fov_y, h, vel, goal, ht, order, vel_hl, RN, do_replanning)
    
    #f.calculate()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
       pass


