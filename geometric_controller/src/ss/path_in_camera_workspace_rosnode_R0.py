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

"""
import random, time, itertools
import numpy as np
from math import sqrt,cos,sin, tan
import networkx as nx
from gudhi import *
import gudhi 
import scipy
from pyquaternion import Quaternion as Qt
from scipy.spatial import cKDTree
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from R0_polynomial_trajectory_generation_no_dynamics_no_corridor import Minimum_snap_trajetory
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import std_msgs.msg
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Point, Quaternion
from nav_msgs.msg import Odometry 
from geometric_controller.msg import PolynomialTrajectory

class trajecotory_in_camera_workspace(object): 
    def __init__(self): 
        """initialization"""
        self.resolution = 0.25
        self.radius_of_obstacle = 0.1315 # just for testing I am using 0.1315 instead of 0.315
        self.sensor_range = 2.5
        self.cone_angle_x = np.pi/3
        self.cone_angle_y = np.pi/4
        self.counter = 0
        self.start_time = time.time()
        self.end_point = np.array([0.0, -5.0, 6.0])
        self.Pglobal = np.array([0,0,0])
        self.Rglobal = np.array([[1,0,0], [0,1,0], [0,0,1]])
        # the rotation matrix between cg of the vehicle and center point of the visensor is obtained 
        # using position (0.1, 0.0, -0.03) and orientation (0.0, 0.049979, 0.0, 0.99875) of visensor in rotorS
        # use translation and rotation of the quad for experiments
        self.Rcg_vibase = np.array([[0.995004, 0.0, 0.0998331, 0.1], [0.0, 1.0, 0.0, 0.0], [-0.0998331, 0.0, 0.995004, -0.03], [0.0, 0.0, 0.0, 1.0]])
        # camera depth optical center link to camera depth link (changes xyz with z point forward to camera , to z pointng upwards)
        self.Rcdoc_cdl = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        # visensor base to camera left link
        self.Rvibase_cl = np.array([[1, 0, 0, 0.015], [0, 1, 0, 0.055], [0, 0, 1, 0.0065], [0, 0, 0, 1]])
        
        self.pub1 = rospy.Publisher('/frequency_of_calculations', Float32, queue_size = 1, tcp_nodelay = True)
        self.uav_traj_pub = rospy.Publisher('/polynomial_trajectory', MultiDOFJointTrajectory, queue_size = 200, tcp_nodelay = True)
        self.custom_uav_traj_pub = rospy.Publisher('/polynomial_trajectory_custom_msg', PolynomialTrajectory, queue_size = 1, tcp_nodelay=True)
        try:
            #rospy.Subscriber('/camera/depth_registered/cloud_for_octomap', PointCloud2, self.callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/firefly/vi_sensor/camera_depth/depth/voxel_points', PointCloud2, self.trajectory_callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.odometry_callback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the topic above')

    def odometry_callback(self, data):
        """get the odometry of the quadrotor"""
        self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.Rglobal = q.rotation_matrix

    def dist(self, p1,p2):
        return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
    
    def generate_regular_pyramid_grid(self): 
        """generate regular grid inside the rectangular pyramid located at uav center and aligned with yaw"""
        z = np.arange(0, self.sensor_range+self.resolution, self.resolution)
        points_in_pyramid = np.zeros((0,3))
        points_in_pyramid = np.concatenate((points_in_pyramid, np.array([[0.0, 0.0, 0.0]])), 0)
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

        points_near_obstacle = kdt_points_in_cone.query_ball_point(p, self.radius_of_obstacle)
        points_to_remove = list(itertools.chain.from_iterable(points_near_obstacle))
        points_in_cone = np.delete(points_in_cone, points_to_remove, 0) 
        return points_in_cone
        
    def plot_graph_and_trajectory(self, points_in_cone, occupied_points, G, path, T, p1, p2, p3): 
        """plot the graph"""
        occupied_points = [np.array([x[0], x[1], x[2]]) for x in occupied_points]
        pp1 = [self.Rvibase_cl[0:3, 3:4] + np.dot(self.Rcdoc_cdl[0:3,0:3], x[np.newaxis].T) for x in occupied_points]
        pp2 = [np.dot(self.Rcg_vibase[0:3, 0:3].T, x) for x in pp1]
        pp3 = [self.Pglobal + np.dot(self.Rglobal.T, x).ravel() for x in pp2]
        op = [x for x in pp3 if not x[2] <= 0]
        op = zip(*op)
        G_edge_list = [(u,v) for (u,v,d) in G.edges(data=True)] 

        fig = plt.figure(figsize=(9,6))
        ax = fig.add_subplot(111, projection='3d')

        for j in G_edge_list: 
            points = np.array([points_in_cone[j[0]], points_in_cone[j[1]]])
            points = zip(*points)
            ax.plot(points[0], points[1], points[2], c = '0.8', linestyle = '--')
            
        ax.plot(path[0], path[1], path[2], c='g')        
        ax.scatter(op[0], op[1], op[2], s = 50, c='b')
    
    
        no_of_segments = len(zip(*path))-1
      
        N = 8
        p1 = [p1[i:i + N] for i in range(0, len(p1), N)]
        [i.reverse() for i in p1]
    
        p2 = [p2[i:i + N] for i in range(0, len(p2), N)]
        [i.reverse() for i in p2]
    
        p3 = [p3[i:i + N] for i in range(0, len(p3), N)]
        [i.reverse() for i in p3]
        
      
        t = []; xx = []; vx = []; aax = []; jx = []; sx = []
        yy = []; vy = []; aay = []; jy = []; sy = []
        zz = []; vz = []; aaz = []; jz = []; sz = []
        #fig = plt.figure(figsize=(9,6))
        #ax = fig.add_subplot(111, projection='3d')
        
        for i in range(no_of_segments): 
            t.append(np.linspace(T[i], T[i+1], 11))   
            xx.append(np.poly1d(p1[i]))
            vx.append(np.polyder(xx[-1], 1)); aax.append(np.polyder(xx[-1], 2))
            jx.append(np.polyder(xx[-1], 3)); sx.append(np.polyder(xx[-1], 4))
            
            yy.append(np.poly1d(p2[i]))
            vy.append(np.polyder(yy[-1], 1)); aay.append(np.polyder(yy[-1], 2))
            jy.append(np.polyder(yy[-1], 3)); sy.append(np.polyder(yy[-1], 4))
    
            zz.append(np.poly1d(p3[i]))
            vz.append(np.polyder(zz[-1], 1)); aaz.append(np.polyder(zz[-1], 2))
            jz.append(np.polyder(zz[-1], 3)); sz.append(np.polyder(zz[-1], 4))
            
        for i in range(no_of_segments): 
            ax.plot(xx[i](t[i]), yy[i](t[i]), zz[i](t[i]), zdir='z', c = 'k')

        plt.xlabel('x', fontsize=24)
        plt.ylabel('y', fontsize=24)
        #plt.legend(loc=3, ncol = 3, fontsize=24)
        plt.xticks(size=16); plt.yticks(size=16)     
        fig = plt.figure()
        velocity_magnitude = []; acceleration_magnitude = []; jerk_magnitude = []
        for j in range(no_of_segments): 
            velocity_magnitude = np.sqrt(vx[j](t[j])**2 + vy[j](t[j])**2 + vz[j](t[j])**2) 
            acceleration_magnitude = np.sqrt(aax[j](t[j])**2 + aay[j](t[j])**2 + aaz[j](t[j])**2)
            #jerk_magnitude = np.sqrt(jx[j](t[j])**2 + jy[j](t[j])**2 + jz[j](t[j])**2)
            plt.plot(t[j], velocity_magnitude, linewidth =2); plt.plot(t[j], acceleration_magnitude, linewidth=2)

        plt.show()
            

    def trajectory_callback(self, data):
        #initialize and prepare screen
        #if self.counter == 0: 
         
        a = time.time()
        occupied_points = ros_numpy.numpify(data)
        points_in_cone = self.generate_regular_pyramid_grid() 
        points_in_cone = self.generate_final_grid(points_in_cone, occupied_points)
        # now convert the points generated in the local frame of the visensor (xyz of the camera where z points outside the camera)
        # to the points in quadrotor cg xyz frame(x forward z upwards) located at the visensor
        print '1', time.time()-a
        p1 = [self.Rvibase_cl[0:3, 3:4] + np.dot(self.Rcdoc_cdl[0:3,0:3], x[np.newaxis].T) for x in points_in_cone]
        p2 = [np.dot(self.Rcg_vibase[0:3, 0:3].T, x) for x in p1]
        points_in_cone = [self.Pglobal + np.dot(self.Rglobal.T, x).ravel() for x in p2]
        points_in_cone = [x for x in points_in_cone if not x[2] <= 0]
        #points_in_cone = [np.dot(self.Rcg_vibase[0:3, 0:3].T, self.Rvibase_cl[0:3, 3:4]+np.dot(self.Rcdoc_cdl[0:3,0:3],x[np.newaxis].T)).ravel() for x in points_in_cone]
        print '2', time.time()-a
        

        kdt_points_in_cone = cKDTree(points_in_cone)
        closest_to_end = kdt_points_in_cone.query(self.end_point, 1)
        print closest_to_end
        #closest_to_end_index = points_in_cone[closest_to_end[1]]
        #closest_to_end_point = (closest_to_end_index[0], closest_to_end_index[1], closest_to_end_index[2]) 
        end_point_index = closest_to_end[1]
        print 'the goal point is', points_in_cone[end_point_index]

        #one dimension simplicial complex which is basically a graph
        rips = gudhi.RipsComplex(points = points_in_cone, max_edge_length = 1.5*self.resolution)
        f = rips.create_simplex_tree(max_dimension = 1)
        
        # make graph
        G = nx.Graph() 
        G.add_nodes_from(range(f.num_vertices()))
        edge_list = [(simplex[0][0], simplex[0][1],  {'weight': simplex[1]}) if len(simplex[0])==2 else None for simplex in f.get_skeleton(1)]
        edge_list = [k for k in edge_list if k is not None]                   
        G.add_edges_from(edge_list)            

        shortest_path = nx.shortest_path(G, source = 0, target = end_point_index, weight = 'weight', method = 'dijkstra')
        path = np.array([points_in_cone[j] for j in shortest_path])
        #length = nx.shortest_path_length(G, source = 0, target = end_point_index, weight = 'weight', method='dijkstra')
        
        # planning horizon is a fixed (5 for now) segments, trajectory will be planned only for these segments, 
        # execution horizon will be just 3 segments
        # at current resolution of 0.25m, that means a trajecotory of approximately 1.25m will be planned and 0.75m will be executed
        # at each time the quad plans its motion, depending on how fast it can replan, these values would be changed
        
        no_of_segments = 5
        path = path[:no_of_segments+1] # 5 segments means 6 points in the path
        path = zip(*path)       
        
                
        
        #no_of_segments = len(path)-1


        # now construct the minimum snap trajectory on the minimum path
        waypoint_specified = True
        waypoint_bc = False
        print path
        T, p1, p2, p3 = Minimum_snap_trajetory(path, waypoint_specified, waypoint_bc).construct_polynomial_trajectory()
        print '8', time.time()-a
        
        #self.plot_graph_and_trajectory(points_in_cone, occupied_points, G, path, T, p1, p2, p3)
        
        
        msg = Float32()
        msg.data = 0.0
        self.pub1.publish(msg)
        
       
        N = 8
        p1 = [p1[i:i + N] for i in range(0, len(p1), N)]
        [i.reverse() for i in p1]
    
        p2 = [p2[i:i + N] for i in range(0, len(p2), N)]
        [i.reverse() for i in p2]
    
        p3 = [p3[i:i + N] for i in range(0, len(p3), N)]
        [i.reverse() for i in p3]
        
      
        t = []; xx = []; yy = []; zz = []
        vx = []; vy = []; vz = []; accx = []; accy = []; accz = []

        print '9', time.time()-a
        traj_freq = 100 # frequency of the trajectory in Hz
        for i in range(no_of_segments): 
            t.append(np.linspace(T[i], T[i+1], int((T[i+1]-T[i])*traj_freq + 1)))   
            xx.append(np.poly1d(p1[i]))
            vx.append(np.polyder(xx[-1], 1)); accx.append(np.polyder(xx[-1], 2))
            #jx.append(np.polyder(xx[-1], 3)); sx.append(np.polyder(xx[-1], 4))
            
            yy.append(np.poly1d(p2[i]))
            vy.append(np.polyder(yy[-1], 1)); accy.append(np.polyder(yy[-1], 2))
            #jy.append(np.polyder(yy[-1], 3)); sy.append(np.polyder(yy[-1], 4))
    
            zz.append(np.poly1d(p3[i]))
            vz.append(np.polyder(zz[-1], 1)); accz.append(np.polyder(zz[-1], 2))
            #jz.append(np.polyder(zz[-1], 3)); sz.append(np.polyder(zz[-1], 4))
            

        traj = MultiDOFJointTrajectory()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'frame'
        traj.header = header
        traj.joint_names = 'base_link'
        
        
        time_diff = no_of_segments*len(t[0])*30
        print 'inverse of time_diff is', 1.0/time_diff    
        
        #epoch = rospy.Time()
        #print epoch, no_of_segments
        #tt = rospy.Time(1, 30)
        #print tt
        msg1 = PolynomialTrajectory()
        for i in range(no_of_segments): 
            for j in t[i]:
                xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                axdes = accx[i](j); aydes = accy[i](j); azdes = accz[i](j)
                
                transforms = Transform(translation = Point(xdes, ydes, zdes), rotation = Quaternion(0,0,0,1))
                velocities = Twist(linear = Point(vxdes, vydes, vzdes), angular = Point(0,0,0))
                accelerations = Twist(linear = Point(axdes, aydes, azdes), angular = Point(0,0,0))
                
                point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(1.0/time_diff))
                traj.points.append(point)    
                #rospy.sleep(1.0/time_diff)
                
                # publish the trajcetory to custom ros message that is used for all other algorithm 
                msg1.pdes.x = xdes; msg1.pdes.y = ydes; msg1.pdes.z = zdes
                msg1.vdes.x = vxdes; msg1.vdes.y = vydes; msg1.vdes.z = vzdes
                msg1.ades.x = axdes; msg1.ades.y = aydes; msg1.ades.z = azdes
                msg1.ddes.x = 1; msg1.ddes.y = 0; msg1.ddes.z = 0
                msg1.controller = 0
                self.custom_uav_traj_pub.publish(msg1)
                f0 = open('trajectory.txt', 'a')
                f0.write("%s, %s, %s\n" % (xdes, ydes, zdes))
                time.sleep(1.0/120)
        f1 = open('trajectory1.txt', 'a')
        f1.write("%s\n" % (traj))
        self.uav_traj_pub.publish(traj)
        print '11', time.time()-a


# if python says run, then we should run
if __name__ == '__main__':
    rospy.init_node('trajectory_in_camera_fov', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(10)    
    traj = trajecotory_in_camera_workspace()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
    
