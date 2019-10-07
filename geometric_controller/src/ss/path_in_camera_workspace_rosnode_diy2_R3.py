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
import time, itertools
import numpy as np
from math import sqrt
import networkx as nx
#from gudhi import *
import gudhi 
#import scipy
from pyquaternion import Quaternion as Qt
from scipy.spatial import cKDTree
from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
from R1_polynomial_trajectory_generation_no_dynamics_no_corridor import Minimum_snap_trajetory
#from R0_3DMST_with_dynamics import Minimum_snap_trajetory
import rospy 
import ros_numpy
from sensor_msgs.msg import PointCloud2
#from std_msgs.msg import Float32
import std_msgs.msg
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Point, Quaternion
from nav_msgs.msg import Odometry 
from geometric_controller.msg import PolynomialCoefficients
import sensor_msgs.point_cloud2 as pc2
from collections import OrderedDict

class trajecotory_in_camera_workspace(object): 
    def __init__(self, name, number, ip, io, res, radius, sr, fov_x, fov_y, h, vel, goal):

        """initialization"""
        open('total_time_taken.txt', 'w').close()
        #open('path.txt', 'w').close()
        #open('eoe_points.txt', 'w').close()
        #open('trajectory_generated.txt', 'w').close()
        self.uav = name
        self.no = str(number)
        self.uav_velocity = vel
        self.resolution = res
        self.radius_of_obstacle = radius
        self.sensor_range = sr
        self.cone_angle_x = fov_x
        self.cone_angle_y = fov_y
        self.traj_gen_counter = 0
        self.planning_counter = 0
        self.hover_counter = 0
        self.hover_start_time = time.time()
        self.hovering_time = time.time()
        self.initial_position = np.asarray(ip)
        self.initial_orientation = np.asarray(io)
        self.hovering_height = h
        self.end_point = np.asarray(goal)#np.array([7, -10, 1.0]) # this is the goal
        self.target_velocity = 1.0
        self.RHP_time = 0.0 # receding horizon planning time, keeps track of the time for the trajectory
        self.Pglobal = np.asarray(ip)
        self.Vglobal = np.array([0,0,0])
        self.Rglobal = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.p_eoe = np.array([[0,0,0]]) # position at the end of execution limit (at the end of 3rd waypoint)
        self.v_eoe = np.array([[0,0,0]]) # velocity at the end of execution limit (at the end of 3rd waypoint)
        self.a_eoe = np.array([[0,0,0]]) # acceleration at the end of execution limit (at the end of 3rd waypoint)
        self.j_eoe = np.array([[0,0,0]]) # jerk at the end of execution limit (at the end of 3rd waypoint)
        # the rotation matrix between cg of the vehicle and center point of the visensor is obtained 
        # using position (0.1, 0.0, -0.03) and orientation (0.0, 0.049979, 0.0, 0.99875) of visensor in rotorS
        # use translation and rotation of the quad for experiments
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
        #self.uav_traj_pub = rospy.Publisher('/'+self.uav+self.no+'/polynomial_trajectory', MultiDOFJointTrajectory, queue_size = 1, tcp_nodelay = True)
        self.target_odom = rospy.Publisher('/'+self.uav+self.no+'/target/odometry', Odometry, queue_size = 1, tcp_nodelay = True)
        self.traj_polycoeff = rospy.Publisher('/firefly1/polynomial_coefficients', PolynomialCoefficients, queue_size = 1, tcp_nodelay = True)
        
        try:
            #rospy.Subscriber('/'+self.uav+'/voxel_points', PointCloud2, self.planning_callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/'+self.uav+self.no+'/vi_sensor/camera_depth/depth/voxel_points', PointCloud2, self.planning_callback, queue_size = 1, tcp_nodelay = True)
            #rospy.Subscriber('/'+self.uav+self.no +'/odom', Odometry, self.odometry_callback, queue_size = 1, tcp_nodelay = True)
            #rospy.Subscriber('/'+self.uav+'/odom', Odometry, self.odometry_callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/'+self.uav+self.no+'/odometry_sensor1/odometry', Odometry, self.odometry_callback, queue_size = 1, tcp_nodelay = True)
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
        #for j in range(len(points_in_pyramid)): 
        #    f7 = open('points_in_cone.txt', 'a')
        #    f7.write('%s, %s, %s, %s\n' %(self.RHP_time, points_in_pyramid[j][0], points_in_pyramid[j][1], points_in_pyramid[j][2]))
        return points_in_pyramid
        
        
    def generate_final_grid(self, points_in_cone, start_point, occupied_points): 
        """generate the final points in the cone by removing obstacles"""
 
        kdt_points_in_cone = cKDTree(points_in_cone)
        

        modified_start_point = np.dot(np.linalg.inv(self.Rglobal), (start_point.T-self.Pglobal[np.newaxis].T))

        modified_start_point = np.dot(self.Rcg_vibase[0:3, 0:3], (modified_start_point+self.Rcg_vibase[0:3, 3:4])) 
        #print '2', modified_start_point
        modified_start_point = np.dot(np.linalg.inv(self.Rcdoc_cdl[0:3,0:3]), modified_start_point)
        #print '3', self.Pglobal, modified_start_point
        
        
        p = np.zeros((occupied_points.shape[0],3))
        p[:,0] = occupied_points['x']
        p[:,1] = occupied_points['y']
        p[:,2] = occupied_points['z']
        # translate points slightly to the visensor link, this may not be needed for real camera
        # the frame is such that the z axis is forward to the camera. 


        pp = [list(self.Rvibase_cl[0:3, 3:4].ravel() + x) for x in p]
        ppp = np.array([i for i in pp])

        kdt_occupied_points = cKDTree(ppp)
        
        points_near_obstacle = kdt_points_in_cone.query_ball_point(pp, self.radius_of_obstacle)
        points_to_remove = list(itertools.chain.from_iterable(points_near_obstacle))
        list_without_duplicate_points = list(OrderedDict.fromkeys(points_to_remove))
        modified_points_in_cone = np.delete(points_in_cone, list_without_duplicate_points, 0)
        
        modified_points_in_cone = np.concatenate((modified_start_point.T, modified_points_in_cone), 0)
        
        kdt_modifed_points = cKDTree(modified_points_in_cone)

        #p1 = [np.dot(self.Rcdoc_cdl[0:3,0:3], x[np.newaxis].T) for x in modified_points_in_cone]
        #p2 = [self.Rcg_vibase[0:3, 3:4] + np.dot(self.Rcg_vibase[0:3, 0:3].T, x) for x in p1]
        #modified_points_in_cone = [self.Pglobal + np.dot(self.Rglobal, x).ravel() for x in p2]
        #modified_points_in_cone = [x for x in modified_points_in_cone if not x[2] <= 0]
        #modified_points_in_cone = np.concatenate((start_point, modified_points_in_cone), 0)       

        
        sparce_matrix = cKDTree.sparse_distance_matrix(kdt_modifed_points, kdt_occupied_points, 50, output_type='ndarray', p = 2)
        weights = [sum(i[2] for i in sparce_matrix[j:j+len(occupied_points)]) for j in range(0,len(sparce_matrix),len(occupied_points))]

        return modified_points_in_cone, weights
       
            
    def trajectory_in_camera_fov(self, data):
        """generates trajecory in camera workspace""" 

        start = time.time()
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.end_point[0]
        odom_msg.pose.pose.position.y = self.end_point[1]
        odom_msg.pose.pose.position.z = self.end_point[2]
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'world'

        self.target_odom.publish(odom_msg)
        t1 = time.time()-start
        
        start_point = np.zeros((0,3))
        if self.traj_gen_counter != 0: 
            start_point = np.concatenate((start_point, self.p_eoe), 0) # was self.p_eoe
        else: 
            start_point = np.concatenate((start_point, self.Pglobal[np.newaxis]), 0) # new axis was needed because you dont maintain uniformity

        #start_point = np.concatenate((start_point, self.Pglobal[np.newaxis]), 0)
        points_in_cone = self.generate_regular_pyramid_grid() 
        occupied_points = ros_numpy.numpify(data)

        if len(occupied_points) != 0: 
            points_in_cone, weights = self.generate_final_grid(points_in_cone, start_point, occupied_points)
            #print weights
            weights = [200.0/i for i in weights] # 200 is just a scale by which weights are increased (after inverting them first)
            #print weights


        p1 = [np.dot(self.Rcdoc_cdl[0:3,0:3], x[np.newaxis].T) for x in points_in_cone]
        p2 = [-self.Rcg_vibase[0:3, 3:4] + np.dot(np.linalg.inv(self.Rcg_vibase[0:3, 0:3]), x) for x in p1]

        points_in_cone = [self.Pglobal + np.dot(self.Rglobal, x).ravel() for x in p2]
        points_in_cone = [x for x in points_in_cone if not x[2] <= 0]

        
        if len(occupied_points) == 0: 
            points_in_cone = np.concatenate((start_point, points_in_cone), 0)
        
        
        # publish generated pointcloud
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        p = pc2.create_cloud_xyz32(header, points_in_cone)
        self.pcl_pub.publish(p)        
        t3 = time.time()-start
        kdt_points_in_cone = cKDTree(points_in_cone)
        closest_to_end = kdt_points_in_cone.query(self.end_point, 1)

        #closest_to_end_index = points_in_cone[closest_to_end[1]]
        #closest_to_end_point = (closest_to_end_index[0], closest_to_end_index[1], closest_to_end_index[2]) 
        end_point_index = closest_to_end[1]


        #one dimension simplicial complex which is basically a graph
        rips = gudhi.RipsComplex(points = points_in_cone, max_edge_length = 1.5*self.resolution)
        f = rips.create_simplex_tree(max_dimension = 1)
        
        # make graph
        G = nx.Graph() 
        G.add_nodes_from(range(f.num_vertices()))
        if len(occupied_points) == 0:
            edge_list = [(simplex[0][0], simplex[0][1],  {'weight': simplex[1]}) if len(simplex[0])==2 else None for simplex in f.get_skeleton(1)]
        else: 

            edge_list = [(simplex[0][0], simplex[0][1],  {'weight': simplex[1] + (weights[simplex[0][0]]+weights[simplex[0][1]])/2}) if len(simplex[0])==2 else None for simplex in f.get_skeleton(1)]
        edge_list = [k for k in edge_list if k is not None]                   
        G.add_edges_from(edge_list)    
        
        try: 
            shortest_path = nx.shortest_path(G, source = 0, target = end_point_index, weight = 'weight', method = 'dijkstra')
            length = nx.shortest_path_length(G, source = 0, target = end_point_index, weight = 'weight', method='dijkstra')
            #print 'length of the path is', length
            path = np.array([points_in_cone[j] for j in shortest_path])
        except: 
            print 'No path between start and end'
        
        #if self.traj_gen_counter == 0 : 
        #    for simplex in f.get_skeleton(1): 
        #        print simplex
        #length = nx.shortest_path_length(G, source = 0, target = end_point_index, weight = 'weight', method='dijkstra')
        
        # planning horizon is a fixed (5 for now) segments, trajectory will be planned only for these segments, 
        # execution horizon will be just 3 segments
        # at current resolution of 0.25m, that means a trajecotory of approximately 1.25m will be planned and 0.75m will be executed
        # at each time the quad plans its motion, depending on how fast it can replan, these values would be changed
        t4 = time.time()-start
        no_of_segments = 4
        no_of_segments_to_track = 1
        path = path[:no_of_segments+1] # n segments means n+1 points in the path
        #f1 = open('path.txt', 'a')
        #f1.write('%s\n' %(path)) 
        path = zip(*path)       

        # now construct the minimum snap trajectory on the minimum path
        waypoint_specified = True
        waypoint_bc = False

        T, _p1, _p2, _p3 = Minimum_snap_trajetory(self.uav_velocity, path, waypoint_specified, waypoint_bc, self.v_eoe, self.a_eoe, self.j_eoe).construct_polynomial_trajectory()
        t5 = time.time()-start
        
       
        #self.plot_graph_and_trajectory(points_in_cone, occupied_points, G, path, T, p1, p2, p3)
        

        N = 8
        _pp1 = [_p1[i:i + N] for i in range(0, len(_p1), N)]
        [i.reverse() for i in _pp1]
    
        _pp2 = [_p2[i:i + N] for i in range(0, len(_p2), N)]
        [i.reverse() for i in _pp2]
    
        _pp3 = [_p3[i:i + N] for i in range(0, len(_p3), N)]
        [i.reverse() for i in _pp3]
        

        xx = np.poly1d(_pp1[no_of_segments_to_track-1])
        vx = np.polyder(xx, 1); accx = np.polyder(xx, 2)
        jx = np.polyder(xx, 3); sx = np.polyder(xx, 4)

        yy = np.poly1d(_pp2[no_of_segments_to_track-1])
        vy = np.polyder(yy, 1); accy = np.polyder(yy, 2)
        jy = np.polyder(yy, 3); sy = np.polyder(yy, 4)
        
        zz = np.poly1d(_pp3[no_of_segments_to_track-1])
        vz = np.polyder(zz, 1); accz = np.polyder(zz, 2)
        jz = np.polyder(zz, 3); sz = np.polyder(zz, 4)
        
        xdes = xx(T[no_of_segments_to_track]); ydes = yy(T[no_of_segments_to_track]); zdes = zz(T[no_of_segments_to_track])
        vxdes = vx(T[no_of_segments_to_track]); vydes = vy(T[no_of_segments_to_track]); vzdes = vz(T[no_of_segments_to_track])
        axdes = accx(T[no_of_segments_to_track]); aydes = accy(T[no_of_segments_to_track]); azdes = accz(T[no_of_segments_to_track])
        jxdes = jx(T[no_of_segments_to_track]); jydes = jy(T[no_of_segments_to_track]); jzdes = jz(T[no_of_segments_to_track])
        """
        t = []; xx = []; yy = []; zz = []
        vx = []; vy = []; vz = []; accx = []; accy = []; accz = []
        jx = []; jy = []; jz = []

        traj_frequency = 100
        for ii in range(no_of_segments_to_track): 

            #t.append(np.linspace(T[ii], T[ii+1], int((T[ii+1]-T[ii])*traj_frequency))) 
            t.append(np.linspace(T[ii], T[ii+1], 101)) 
            xx.append(np.poly1d(p1[ii]))
            vx.append(np.polyder(xx[-1], 1)); accx.append(np.polyder(xx[-1], 2))
            jx.append(np.polyder(xx[-1], 3))#; sx.append(np.polyder(xx[-1], 4))
            
            yy.append(np.poly1d(p2[ii]))
            vy.append(np.polyder(yy[-1], 1)); accy.append(np.polyder(yy[-1], 2))
            jy.append(np.polyder(yy[-1], 3))#; sy.append(np.polyder(yy[-1], 4))
    
            zz.append(np.poly1d(p3[ii]))
            vz.append(np.polyder(zz[-1], 1)); accz.append(np.polyder(zz[-1], 2))
            jz.append(np.polyder(zz[-1], 3))#; sz.append(np.polyder(zz[-1], 4))
            
        t6 = time.time()-start
        traj = MultiDOFJointTrajectory()
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        traj.header = header
        traj.joint_names = 'firefly/base_link' # testing for now
        
        
        trajectory_start_time = self.RHP_time
        for i in range(no_of_segments_to_track): # "changed to no_of_segments_to_track" instead of "no_of_segments" 
            for j in t[i]:
                    
                self.RHP_time = j + trajectory_start_time    
                xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
                jxdes = jx[i](j); jydes = jy[i](j); jzdes = jz[i](j)
                f4 = open('trajectory_generated.txt','a')
                f4.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (self.RHP_time, xdes, ydes, zdes, vxdes, vydes, vzdes, axdes, aydes, azdes))
                    
                
                # for now angular acceleration Point msg of MultiDOFJointTrajectory() msg is  used to specify the desired direction 
                #vector = np.array([self.end_point[0]-xdes, self.end_point[1]-ydes, self.end_point[2]-zdes])
                #vector = np.array([self.p_eoe[0][0]-xdes, self.p_eoe[0][1]-ydes, self.p_eoe[0][2]-zdes])
                vector = np.array([self.end_point[0]-self.Pglobal[0], self.end_point[1]-self.Pglobal[1], self.end_point[2]-self.Pglobal[2]])
                #vector = np.array([self.Vglobal[0], self.Vglobal[1], self.Vglobal[2]])
                direction = vector/np.linalg.norm(vector)
                
                transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(direction[0],direction[1],direction[2]))
                #accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(1,0,0))
                
                point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(self.RHP_time))
                traj.points.append(point)
                self.uav_traj_pub.publish(traj)
                traj.points.pop(0)  
                #time.sleep(1.0/300)
        """
        self.p_eoe = np.array([[xdes, ydes, zdes]])
        self.v_eoe = np.array([[vxdes, vydes, vzdes]])
        self.a_eoe = np.array([[axdes, aydes, azdes]])
        self.j_eoe = np.array([[jxdes, jydes, jzdes]])
        #f1 = open('eoe_points.txt', 'a')
        #f1.write('%s, %s, %s, %s\n' %(self.p_eoe, self.v_eoe, self.a_eoe, self.j_eoe)) 
        

        #self.uav_traj_pub.publish(traj)
        
        self.traj_gen_counter += 1


        vector = np.array([self.end_point[0]-self.Pglobal[0], self.end_point[1]-self.Pglobal[1], self.end_point[2]-self.Pglobal[2]])
        direction = vector/np.linalg.norm(vector)
        
        msg = PolynomialCoefficients()
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        msg.header = header
        msg.polynomial_order = N-1
        for i in range(len(_p1)): 
            msg.poly_x.append(_p1[i]); msg.poly_y.append(_p2[i]); msg.poly_z.append(_p3[i])
        msg.number_of_segments = no_of_segments_to_track
        msg.planning_start_time = data.header.stamp.to_sec()
        for j in T[:no_of_segments_to_track+1]: 
            msg.segment_end_times.append(j)
        msg.desired_direction.x = direction[0]; msg.desired_direction.y = direction[1]; msg.desired_direction.z = direction[2]
        self.traj_polycoeff.publish(msg)
        
        
        time_taken = time.time()-start
        #time.sleep(T[no_of_segments_to_track]-0.1-time_taken)
        self.RHP_time = data.header.stamp.to_sec()-self.hover_start_time
        
        print 'total time taken to execute the callbacak is:', time_taken

        f1 = open('total_time_taken.txt', 'a')
        f1.write("%s, %s, %s, %s, %s, %s, %s\n" % (T[no_of_segments_to_track], start, t1, t3, t4, t5, time_taken))  
    def hover_the_quad(self, data): 
        """give commands to hover at 1m above the starting position"""
        a = time.time()
        if self.hover_counter == 0: 
            self.hover_start_time = data.header.stamp.to_sec()
        self.RHP_time = data.header.stamp.to_sec()-self.hover_start_time


        waypoint_specified = True
        waypoint_bc = False
        path = [[self.initial_position[0], self.initial_position[1], self.initial_position[2]], [self.initial_position[0], self.initial_position[1], self.hovering_height]]
        path = zip(*path)
        vel = np.array([[0,0,0]]); acc = np.array([[0,0,0]]); jerk = np.array([[0,0,0]])
        velocity = 1
        T, p1, p2, p3 = Minimum_snap_trajetory(velocity, path, waypoint_specified, waypoint_bc, vel, acc, jerk).construct_polynomial_trajectory()

        msg = PolynomialCoefficients()
        header = std_msgs.msg.Header()
        header.stamp =  data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        msg.header = header
        msg.polynomial_order = 7
        for i in range(len(p1)): 
            msg.poly_x.append(p1[i]); msg.poly_y.append(p2[i]); msg.poly_z.append(p3[i])
        msg.number_of_segments = 1
        msg.planning_start_time =  self.hover_start_time
        for j in T: 
            msg.segment_end_times.append(j)
        msg.desired_direction.x = 1; msg.desired_direction.y = 0; msg.desired_direction.z = 0
        self.traj_polycoeff.publish(msg)
        f1 = open('total_time_taken.txt', 'a')
        f1.write("%s,\n" % (time.time()-a))
        self.hover_counter += 1
        """
        # for now angular acceleration Point is used to specify the direction 
        traj = MultiDOFJointTrajectory()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        traj.header = header
        traj.joint_names = 'firefly/base_link'  
        
        #print self.initial_position, self.initial_orientation, self.hovering_height
        transforms = Transform(translation = Point(self.initial_position[0], self.initial_position[1], self.hovering_height), rotation = Quaternion(0,0,0,1))
        velocities = Twist(linear = Point(0, 0, 0), angular = Point(0,0,0))
        accelerations = Twist(linear = Point(0, 0, 0), angular = Point(self.initial_orientation[0],self.initial_orientation[1],self.initial_orientation[2]))
        
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(self.RHP_time))
        traj.points.append(point) 
        self.uav_traj_pub.publish(traj)
        """       
        
    def planning_callback(self, data): 
        """main motion planner, plan different trajectories for different stages"""
        #print 'times are', data.header.stamp.to_sec(), self.RHP_time
        if self.RHP_time <= 5.0: 
            self.hover_the_quad(data)
            #time.sleep(3.0)
        else: 
            self.trajectory_in_camera_fov(data)
        
        self.planning_counter += 1

# if python says run, then we should run
if __name__ == '__main__':
    
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
    vel = rospy.get_param('uav_velocity')
    goal = rospy.get_param('goal_point')
    rospy.init_node('trajectory_in_camera_fov', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(30)    
    traj = trajecotory_in_camera_workspace(name, number, initial_position, initial_orientation, res, radius, sr, fov_x, fov_y, h, vel, goal)
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
    
