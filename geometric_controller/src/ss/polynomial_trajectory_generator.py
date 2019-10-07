#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  5 14:01:09 2019
Minimum snap trajectory generation algorithm based on Rithcer's paper 
R1: takes multiple segments
@author: indrajeet
"""

import numpy as np 
from matplotlib import pyplot as plt 
import time, operator, scipy
#from mpl_toolkits.mplot3d import Axes3D
#import os, time, operator, json
from gurobipy import *
from functools import reduce
#from multiprocessing import Process
from itertools import chain
from scipy.linalg import block_diag
import rospy
from nav_msgs.msg import Odometry
from geometric_controller.msg import PolynomialTrajectory 
from geometry_msgs.msg import Pose


class Polynomial_trajetory_generator(object):
    def __init__(self, name, freq, waypoint_specified, waypoint_bc): 
        """initializes variables, just does for 4 to start with"""
        self.dt = 1.0/freq
        self.uav = name
        self.wp_specfied = waypoint_specified
        self.wp_bc = waypoint_bc
        self.start_time = time.time()
        self.average_speed = 3.0
        self.reduced_speed = 0.5
        # specify start/intermediate/end points and its deviratives 
        self.no_of_segments = 7
        self.wp_callback_counter = 0
        self.trajectory_constructed = False

        self.r = 4 # corresponding to snap which is 4th  derivative
        self.N = 7# degree of polynomial 
        
        self.pub = rospy.Publisher('/'+self.uav+'/PolynomialTrajectory', PolynomialTrajectory, queue_size = 1, tcp_nodelay = True)
        
        rospy.Subscriber('/'+self.uav + '/odometry_sensor1/odometry', Odometry, self.currentstate_callback, queue_size = 1, tcp_nodelay = True)        
        rospy.Subscriber('/'+self.uav+'/waypoint_publisher', Pose, self.waypoint_callback, queue_size = 1, tcp_nodelay=True)

        #try: 
        #    rospy.Subscriber('/'+self.uav+'/waypoint_publisher', Pose, self.waypoint_callback, queue_size = 1, tcp_nodelay=True)
        #    rospy.Subscriber('/'+self.uav + '/odometry_sensor/odometry', Odometry, self.currentstate_callback, queue_size = 1, tcp_nodelay = True)
        #except: 
        #    print 'Either waypoints or odometry is not available.'
            
    def waypoint_callback(self, wp): 
        """callback to get waypoint positions"""
        if self.trajectory_constructed == False: 
            NextwpPosition = np.array([wp.position.x, wp.position.y, wp.position.z])
            NextwpOrientation = np.array([wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w])
            self.pc_x, self.pc_y, self.pc_z, self.seg_times, self.traj_t0 = self.make_trajectory(NextwpPosition, NextwpOrientation)            
            self.trajectory_constructed = True
    
    def currentstate_callback(self, odom): 
        """this callback gives robot its current position"""
        self.CurrentPosition = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        self.CurrentVelocity = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])
        
    def prod(self, iterable):
        return reduce(operator.mul, iterable, 1)
         
    def prodoftwoterms(self, ii, ll, r): 
        """function to iterate the product of two terms"""
        prod = 1
        for k in range(r):
            prod = prod * (ii-k) * (ll-k) 
        return prod

    
    def construct_Q(self, N, r, t0, tf): 
        """function to constructs the Q matrix"""
        q = np.zeros((N, N))
        for i in xrange(N): 
            for l in xrange(N):
                if i>= r and l>=r:  
                    q[i,l] = 2*self.prodoftwoterms(i,l,r)*(tf**(i+l-2*r+1) - t0**(i+l-2*r+1))/(i+l-2*r+1)
        return q
      
    def construct_A(self, N, r, t0, tf): 
        """function to construct the A matrix"""
        a = np.zeros((2*r,N))
                   
        for i in range(N): 
            for j in range(r): 
                if i >= j : 
                    a[j,i] = self.prod(xrange(i-(j-1), i+1))*t0**(i-j)
                    a[r+j,i] = self.prod(xrange(i-(j-1), i+1))*tf**(i-j)
        return a
    
    def construct_b(self, start, end): 
        """function that makes vector b of ONLY end points"""
        return np.concatenate((start, end), axis = 0)

        
    def publish(self): 
        """function to plot position, velocity, acceleration, jerk and snap"""
        msg = PolynomialTrajectory()

        if self.trajectory_constructed == True: 
            t = time.time()-self.traj_t0

            if t <= self.seg_times[-1]: 
                segment = map(lambda x:x > t, self.seg_times).index(True)
            else: 
                segment = self.no_of_segments; t = self.seg_times[-1]
                
            rospy.loginfo('the value of t and segment is:%f, %d', t, segment)
            #rospy.loginfo('segment times are:%f, %f, %f, %f, %f, %f', self.segment_times[0], \
            #self.segment_times[1], self.segment_times[2], self.segment_times[3], self.segment_times[4], self.segment_times[5])
            p1 = self.pc_x; p2 = self.pc_y; p3 = self.pc_z
            N = self.N+1
            p1 = [p1[i:i + N] for i in range(0, len(p1), N)]
            [i.reverse() for i in p1]
        
            p2 = [p2[i:i + N] for i in range(0, len(p2), N)]
            [i.reverse() for i in p2]
        
            p3 = [p3[i:i + N] for i in range(0, len(p3), N)]
            [i.reverse() for i in p3]
            xx = np.poly1d(p1[segment-1]); vx = np.polyder(xx, 1); ax = np.polyder(xx, 2)
            yy = np.poly1d(p2[segment-1]); vy = np.polyder(yy, 1); ay = np.polyder(yy, 2)
            zz = np.poly1d(p3[segment-1]); vz = np.polyder(zz, 1); az = np.polyder(zz, 2)
            
            msg.header.stamp = rospy.Time.now()
            msg.pdes.x = xx(t); msg.pdes.y = yy(t); msg.pdes.z = zz(t)
            msg.vdes.x = vx(t); msg.vdes.y = vy(t); msg.ades.z = vz(t)
            msg.ades.x = ax(t); msg.ades.y = ay(t); msg.ades.z = az(t)
            msg.ddes.x = 1; msg.ddes.y = 0; msg.ddes.z = 0
            msg.controller = 0
            
            self.pub.publish(msg)

        else: 
            rospy.loginfo('no goal is published yet')

    def make_trajectory(self, NextwpPosition, NextwpOrientation):
        """ function to construct the trajectory"""
        d = np.linalg.norm(self.CurrentPosition - NextwpPosition)
        inter_segment_distance = 1
        self.no_of_segments = 1+int(d//inter_segment_distance)
        

        # enter sequence of waypoints: no of points should be self.no_of_segments+1
        x_wp = np.linspace(self.CurrentPosition[0], NextwpPosition[0], self.no_of_segments+1)
        y_wp = np.linspace(self.CurrentPosition[1], NextwpPosition[1], self.no_of_segments+1)
        z_wp = np.linspace(self.CurrentPosition[2], NextwpPosition[2], self.no_of_segments+1)
        
        # add intial and final condiions vel, acc, jerk
        x_ic = np.array([0, 0, 0])
        x_fc = np.array([0, 0, 0])
        x0 = np.array([x_wp[0], x_ic[0], x_ic[1], x_ic[2]])
        xT = np.array([x_wp[-1], x_fc[0], x_fc[1], x_fc[2]])

        y_ic = np.array([0, 0, 0])
        y_fc = np.array([0, 0, 0])
        y0 = np.array([y_wp[0], y_ic[0], y_ic[1], y_ic[2]])
        yT = np.array([y_wp[-1], y_fc[0], y_fc[1], y_fc[2]])
        
        z_ic = np.array([0, 0, 0])
        z_fc = np.array([0, 0, 0])
        z0 = np.array([z_wp[0], z_ic[0], z_ic[1], z_ic[2]])
        zT = np.array([z_wp[-1], z_fc[0], z_fc[1], z_fc[2]])

        path = [np.sqrt((x_wp[i]-x_wp[i-1])**2 + (y_wp[i]-y_wp[i-1])**2 + (z_wp[i]-z_wp[i-1])**2) for i in range(1, self.no_of_segments+1, 1)]

    
        T = []; T.insert(0, 0)
        T.insert(1, T[-1] + path[0]/self.reduced_speed)
        for i in range(1, len(path)-1, 1):
                T.append(T[-1] + path[i]/self.average_speed)
        T.insert(len(T)+1, T[-1]+path[-1]/self.reduced_speed)   




        #T = []; T.insert(0, 0) # insert 0 at 0 position
        #for i in range(self.no_of_segments): 
        #   T.append(T[-1]+path[i]/self.average_speed)

        r = self.r
        N = 1 + self.N # because number of terms in a polynomial = degree+1

        QQ = []; AA_inv = []

        for i in range(self.no_of_segments): 
            q = self.construct_Q(N, r, T[i], T[i+1])
            a = self.construct_A(N, r, T[i], T[i+1])
            a_inv = scipy.linalg.pinv(a)
            QQ = block_diag(QQ, q)
            AA_inv = block_diag(AA_inv, a_inv)
  
        order = 2*r*self.no_of_segments
        R = np.dot(AA_inv.T, np.dot(QQ, AA_inv))
        
        bx = self.construct_b(x0, xT)
        by = self.construct_b(y0, yT)
        bz = self.construct_b(z0, zT)

        m = Model("qp")
        order = 2*r*self.no_of_segments
        dx = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dx")
        dy = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dy")        
        dz = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dz")        

        # making objective using quicksum, takes a lot of time 
        #obj1 = quicksum(dx[i] * quicksum(R[i][j] * dx[j] for j in range(order)) for i in range(order))
        #obj2 = quicksum(dy[i] * quicksum(R[i][j] * dy[j] for j in range(order)) for i in range(order))
        #obj3 = quicksum(dz[i] * quicksum(R[i][j] * dz[j] for j in range(order)) for i in range(order))
        
        # using LinExpr for the second expression is significantly faster 
        obj1 = quicksum(dx[i] * LinExpr([(R[i][j], dx[j]) for j in range(order)]) for i in range(order))
        obj2 = quicksum(dy[i] * LinExpr([(R[i][j], dy[j]) for j in range(order)]) for i in range(order))
        obj3 = quicksum(dz[i] * LinExpr([(R[i][j], dz[j]) for j in range(order)]) for i in range(order))
        obj = obj1 + obj2 + obj3
        j = 0
        for i in range(order):             
            if i < r: 
                m.addConstr(dx[i] == bx[i])
                m.addConstr(dy[i] == by[i])
                m.addConstr(dz[i] == bz[i])
            elif i >= order-r:                 
                m.addConstr(dx[i] == bx[r+j])
                m.addConstr(dy[i] == by[r+j])
                m.addConstr(dz[i] == bz[r+j])
                j += 1
                
        c = 1 # counter
        for i in range(r, order-2*r, 2*r): 
            #m.addConstr(dx[i] == self.x_wp[c])
            #m.addConstr(dy[i] == self.y_wp[c])
            #m.addConstr(dz[i] == self.z_wp[c])
            m.addConstr(dx[i] <= x_wp[c] + 0.2)
            m.addConstr(dx[i] >= x_wp[c] - 0.2)
            m.addConstr(dy[i] <= y_wp[c] + 0.2)
            m.addConstr(dy[i] >= y_wp[c] - 0.2)
            m.addConstr(dz[i] <= z_wp[c] + 0.2)
            m.addConstr(dz[i] >= z_wp[c] - 0.2)
            c = c+1
            for j in range(r): 
                m.addConstr(dx[i+j] == dx[i+j+r])
                m.addConstr(dy[i+j] == dy[i+j+r])
                m.addConstr(dz[i+j] == dz[i+j+r])
                #if j ==2: 
                #    m.addConstr(dx[i+j] == 2.0)

        m.setObjective(obj, GRB.MINIMIZE)
        #m.write('model.lp')
        m.setParam('OutputFlag', 0)
        m.setParam('PSDtol', 1e-1)
        m.optimize()


        runtime = m.Runtime


        x_coeff = [dx[i].X for i in range(order)]
        y_coeff = [dy[i].X for i in range(order)]
        z_coeff = [dz[i].X for i in range(order)]

        Dx = np.asarray(x_coeff)[np.newaxis].T
        Dy = np.asarray(y_coeff)[np.newaxis].T        
        Dz = np.asarray(z_coeff)[np.newaxis].T        
        pcx = np.dot(AA_inv, Dx); pcy = np.dot(AA_inv, Dy); pcz = np.dot(AA_inv, Dz)


        poly_coeff_x = pcx.T.ravel().tolist()
        poly_coeff_y = pcy.T.ravel().tolist()
        poly_coeff_z = pcz.T.ravel().tolist()

        return poly_coeff_x, poly_coeff_y, poly_coeff_z, T, time.time()
        #self.publish(poly_coeff_x, poly_coeff_y, poly_coeff_z)


if __name__ == '__main__':
    name = 'firefly'#rospy.get_param('robot_name')
    rospy.init_node('polynomial_trajectory', anonymous=True, log_level=rospy.DEBUG)
    waypoint_specified = True
    waypoint_bc = False
    freq = 100.0
    #rospy.init_node('publish_targets', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(freq)
    f = Polynomial_trajetory_generator(name, freq, waypoint_specified, waypoint_bc)
    try:
        while not rospy.is_shutdown():             
            f.publish()
            r.sleep()
    except rospy.ROSInterruptException(): pass