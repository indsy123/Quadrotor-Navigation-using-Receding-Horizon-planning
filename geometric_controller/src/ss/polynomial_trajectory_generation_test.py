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
import time, operator, scipy, math, rospy
#from mpl_toolkits.mplot3d import Axes3D
#import os, time, operator, json
from gurobipy import *
from functools import reduce
#from multiprocessing import Process
from itertools import chain
from scipy.linalg import block_diag
from mpl_toolkits.mplot3d import Axes3D
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import std_msgs.msg
from nav_msgs.msg import Odometry 
from pyquaternion import Quaternion as Qt
from geometry_msgs.msg import Twist, Transform, Point, Quaternion
from geometric_controller.msg import PolynomialCoefficients

class Minimum_snap_trajetory(object):
    def __init__(self, path, waypoint_specified, waypoint_bc): 
        """initializes variables, just does for 4 to start with"""
        open("trajectory_generated.txt", "w").close()
        self.counter = 0
        self.wp_specfied = waypoint_specified
        self.wp_bc = waypoint_bc
        #self.BC_at_waypoints = np.array[]
        self.end_point = np.asarray(path[-1])
        # specify start/intermediate/end points and its deviratives 
        self.hover_counter = 0
        self.start_time = time.time()
        path = zip(*path)
        self.x_wp = np.array(path[0])
        self.y_wp = np.array(path[1])
        self.z_wp = np.array(path[2])
        #print self.x_wp, self.y_wp, self.z_wp
        self.no_of_segments = len(self.x_wp)-1
        #print self.no_of_segments
        # enter sequence of waypoints: no of points should be self.no_of_segments+1
        #self.x_wp = np.linspace(0, 2.0, self.no_of_segments+1)
        #self.x_wp = [0, 0.4, 1, 1.6, 2.4, 3]
        # add intial and final condiions vel, acc, jerk
        self.x_ic = np.array([0, 0, 0])
        self.x_fc = np.array([0, 0, 0])
        self.x0 = np.array([self.x_wp[0], self.x_ic[0], self.x_ic[1], self.x_ic[2]])
        self.xT = np.array([self.x_wp[-1], self.x_fc[0], self.x_fc[1], self.x_fc[2]])

        #self.y_wp = np.linspace(0, 2.0, self.no_of_segments+1)
        #self.y_wp = [0, 0.8, 1, 1.7, 2.1, 2.8]
        # add intial and final condiions vel, acc, jek
        self.y_ic = np.array([0, 0, 0])
        self.y_fc = np.array([0, 0, 0])
        self.y0 = np.array([self.y_wp[0], self.y_ic[0], self.y_ic[1], self.y_ic[2]])
        self.yT = np.array([self.y_wp[-1], self.y_fc[0], self.y_fc[1], self.y_fc[2]])

        #self.z_wp = np.linspace(0, 3.0, self.no_of_segments+1)
        #self.z_wp = [0, 0.8, 1.6, 2.4, 3.2, 4]
        # add initial and final conditions vel, acc, jerk
        self.z_ic = np.array([0, 0, 0])
        self.z_fc = np.array([0, 0, 0])
        self.z0 = np.array([self.z_wp[0], self.z_ic[0], self.z_ic[1], self.z_ic[2]])
        self.zT = np.array([self.z_wp[-1], self.z_fc[0], self.z_fc[1], self.z_fc[2]])
        # now enter end times for each segment 
        self.path = [np.sqrt((self.x_wp[i]-self.x_wp[i-1])**2 + (self.y_wp[i]-self.y_wp[i-1])**2 + \
        (self.z_wp[i]-self.z_wp[i-1])**2) for i in range(1, self.no_of_segments+1, 1)]
        self.T = []; self.T.insert(0, 0)
        


        self.T = []; self.T.insert(0, 0)
        vel = 1.5
        planning_counter = -1
        if len(self.path) == 0: 
            print 'there is no path..'
        if planning_counter == -1: 
            acc_low = 0.5*vel
            self.T.insert(1, self.T[-1]+np.sqrt(2*self.path[0]/acc_low))           
            
        if planning_counter == 0: 
            if len(self.path) == 1: 
                acc_low = 0.5*vel
                self.T.insert(1, self.T[-1]+np.sqrt(2*self.path[0]/acc_low))
            elif len(self.path) == 2: 
                acc_low = 0.5*vel
                acc = vel
                self.T.insert(1, self.T[-1]+np.sqrt(2*self.path[0]/acc_low))
                self.T.append(self.T[-1]+np.sqrt(2*self.path[1]/acc))
            else: 
                acc_low = 0.5*vel
                acc = vel    
                self.T.insert(1, self.T[-1]+np.sqrt(2*self.path[0]/acc_low))
                for i in range(1, len(self.path)-1, 1):
                        self.T.append(self.T[-1]+np.sqrt(2*self.path[i]/acc))
                self.T.append(self.T[-1]+np.sqrt(2*self.path[-1]/acc_low))
        elif planning_counter > 0: 
            acc = vel
            for i in range(0, len(self.path), 1):
                self.T.append(self.T[-1]+np.sqrt(2*self.path[i]/acc))
                
        print len(self.path), planning_counter, self.T 

        
        #low_speed = 1
        #average_speed = 3
        #self.T.insert(1, self.T[-1]+self.path[0]/low_speed)
        #for i in range(1, len(self.path)-1, 1):
        #        self.T.append(self.T[-1]+self.path[i]/average_speed)
        #self.T.insert(len(self.T)+1, self.T[-1]+self.path[-1]/low_speed) 
        #self.T = [i/self.T[-1] for i in self.T]
        #acc_low = 2
        #acc_av = 10        
        #self.T.insert(1, self.T[-1]+np.sqrt(2*self.path[0]/acc_low))
        #for i in range(1, len(self.path)-1, 1):
        #        self.T.append(self.T[-1]+np.sqrt(2*self.path[i]/acc_av))
        #self.T.insert(len(self.T)+1, self.T[-1]+np.sqrt(2*self.path[-1]/acc_low))         
        #print self.path, self.T
        self.r = 4 # corresponding to snap which is 4th  derivative
        self.N = 7# degree of polynomial 
        self.RHP_time = 0
        #self.uav_traj_pub = rospy.Publisher('/firefly1/polynomial_trajectory', MultiDOFJointTrajectory, queue_size = 1, tcp_nodelay = True)
        self.traj_polycoeff = rospy.Publisher('/firefly1/polynomial_coefficients', PolynomialCoefficients, queue_size = 1, tcp_nodelay = True)

        rospy.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, self.callback, tcp_nodelay = True)
        rospy.spin()
    def prod(self, iterable):
        return reduce(operator.mul, iterable, 1)
         
    def prodoftwoterms(self, ii, ll, r): 
        """fucntion to iterate the product of two terms"""
        prod = 1
        for k in range(r):
            prod = prod * (ii-k)*(ll-k) 
        return prod

    
    def construct_Q(self, N, r, t0, tf): 
        """function to constructs the Q matrix"""
        q = np.zeros((N, N))
        for i in xrange(N): 
            for l in xrange(2*r):
                if i>= r and l>=r:  
                    #a = math.exp((i+l-2*r+1)*math.log(tf)); b = math.exp((i+l-2*r+1)*math.log(t0))
                    #q[i,l] = 2*self.prodoftwoterms(i,l,r)*(tf**(i+l-2*r+1) - t0**(i+l-2*r+1))/(i+l-2*r+1)
                    q[i,l] = 2*self.prodoftwoterms(i,l,r)*(math.pow(tf, i+l-2*r+1) - math.pow(t0, i+l-2*r+1))/(i+l-2*r+1)
                    #q[i,l] = 2*self.prodoftwoterms(i,l,r)*(a - b)/(i+l-2*r+1)

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
    
        
    def construct_DF(self, order, r): 
        """function that makes vector DF of ONLY constrained end points"""
        l = [[i] for i in self.x_wp[1:-1]]
        l = list(chain.from_iterable(l))
        DF = np.concatenate((self.x0, l, self.xT), axis = 0)
        return DF
        

    def plot(self, pp1, pp2, pp3, data): 
        """function to plot position, velocity, acceleration, jerk and snap"""
        """
        N = self.N+1
       
        p1 = [pp1[i:i + N] for i in range(0, len(pp1), N)]
        [i.reverse() for i in p1]
    
        p2 = [pp2[i:i + N] for i in range(0, len(pp2), N)]
        [i.reverse() for i in p2]
    
        p3 = [pp3[i:i + N] for i in range(0, len(pp3), N)]
        [i.reverse() for i in p3]
        
        t = []; xx = []; vx = []; aax = []; jx = []; sx = []
        yy = []; vy = []; aay = []; jy = []; sy = []
        zz = []; vz = []; aaz = []; jz = []; sz = []
        
        
        
        for i in range(self.no_of_segments): 
            t.append(np.linspace(self.T[i], self.T[i+1], 101))   
            xx.append(np.poly1d(p1[i]))
            vx.append(np.polyder(xx[-1], 1)); aax.append(np.polyder(xx[-1], 2))
            jx.append(np.polyder(xx[-1], 3)); sx.append(np.polyder(xx[-1], 4))
            
            yy.append(np.poly1d(p2[i]))
            vy.append(np.polyder(yy[-1], 1)); aay.append(np.polyder(yy[-1], 2))
            jy.append(np.polyder(yy[-1], 3)); sy.append(np.polyder(yy[-1], 4))
    
            zz.append(np.poly1d(p3[i]))
            vz.append(np.polyder(zz[-1], 1)); aaz.append(np.polyder(zz[-1], 2))
            jz.append(np.polyder(zz[-1], 3)); sz.append(np.polyder(zz[-1], 4))

        traj = MultiDOFJointTrajectory()
        header = std_msgs.msg.Header()
        header.stamp = data.header.stamp#rospy.Time.now()
        header.frame_id = 'world'
        traj.header = header
        traj.joint_names = 'nothing' # testing for now
        
        
        trajectory_start_time = self.RHP_time
        for i in range(self.no_of_segments): # "changed to no_of_segments_to_track" instead of "no_of_segments" 
            for j in t[i]:
                    
                self.RHP_time = j + trajectory_start_time    
                xdes = xx[i](j); ydes = yy[i](j); zdes = zz[i](j)
                vxdes = vx[i](j); vydes = vy[i](j); vzdes = vz[i](j)
                axdes = aax[i](j); aydes = aay[i](j); azdes = aaz[i](j)
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
                
                #self.uav_traj_pub.publish(traj)
                traj.points.pop(0) 
        
        """


    def callback(self, data):
        """ function to construct the trajectory
        polynmial is of the form x = c0+c1*t+c2*t**2+...cn*t**n
        """
        
        
        if self.counter == 0: 
            self.Pglobal = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
            q = Qt(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
            self.Rglobal = q.rotation_matrix
            V = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
            self.Vglobal = np.dot(self.Rglobal, V)
            
            t = time.time()
            r = self.r
            N = 1 + self.N # because number of terms in a polynomial = degree+1  
    
            QQ = []; AA_inv = []
            #self.T = [0, 1]
            #self.no_of_segments = 1
            for i in range(self.no_of_segments): 
                q = self.construct_Q(N, r, self.T[i], self.T[i+1])
                a = self.construct_A(N, r, self.T[i], self.T[i+1])
                a_inv = scipy.linalg.pinv(a)
                QQ = block_diag(QQ, q)
                AA_inv = block_diag(AA_inv, a_inv)
                #print 'a', a
            
            order = 2*r*self.no_of_segments
            R = np.dot(AA_inv.T, np.dot(QQ, AA_inv))   
            
            bx = np.concatenate((self.x0, self.xT), axis = 0)
            by = np.concatenate((self.y0, self.yT), axis = 0)
            bz = np.concatenate((self.z0, self.zT), axis = 0)
    
            m = Model("qcp")
            order = 2*r*self.no_of_segments
    
            dx = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dx")
            dy = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dy")        
            dz = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dz")    
    
    
            # using LinExpr for the second expression is significantly faster 
            obj1 = quicksum(dx[i] * LinExpr([(R[i][j], dx[j]) for j in range(order)]) for i in range(order))
            obj2 = quicksum(dy[i] * LinExpr([(R[i][j], dy[j]) for j in range(order)]) for i in range(order))
            obj3 = quicksum(dz[i] * LinExpr([(R[i][j], dz[j]) for j in range(order)]) for i in range(order))
    
            
            obj = obj1 + obj2 + obj3 #+ quicksum(T[i] for i in self.no_of_segments)
            
    
            j = 0
            for i in range(order):             
                if i < r: # was r in stead of 1 
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
                m.addConstr(dx[i] == self.x_wp[c])
                m.addConstr(dy[i] == self.y_wp[c])
                m.addConstr(dz[i] == self.z_wp[c])
    
                c = c+1
                for j in range(r): 
                    m.addConstr(dx[i+j] == dx[i+j+r])
                    m.addConstr(dy[i+j] == dy[i+j+r])
                    m.addConstr(dz[i+j] == dz[i+j+r])
    
    
            m.setObjective(obj, GRB.MINIMIZE)
            #m.write('model.lp')
            m.setParam('OutputFlag', 0)
            m.setParam('PSDtol', 1e-3)
            m.setParam('NumericFocus', 3)
            m.optimize()
    
    
            runtime = m.Runtime
            optimal_objective = obj.getValue()
            #print 'optimal objective is:', optimal_objective
    
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
            
            if self.hover_counter == 0: 
                self.hover_start_time = data.header.stamp.to_sec()

            msg = PolynomialCoefficients()
            header = std_msgs.msg.Header()
            header.stamp = data.header.stamp#rospy.Time.now()
            header.frame_id = 'world'
            msg.header = header
            msg.polynomial_order = self.N
            for i in range(len(poly_coeff_x)): 
                msg.poly_x.append(poly_coeff_x[i]); msg.poly_y.append(poly_coeff_y[i]); msg.poly_z.append(poly_coeff_z[i])
            msg.number_of_segments = self.no_of_segments
            msg.planning_start_time = self.hover_start_time
            for j in self.T: 
                msg.segment_end_times.append(j)
            msg.desired_direction.x = 1; msg.desired_direction.y = 0; msg.desired_direction.z = 0
            self.traj_polycoeff.publish(msg)
            #time.sleep(self.T[self.no_of_segments]*0.9)
        else: 
            print "now doing nothing"
        #self.counter += 1
        self.hover_counter += 1
        
if __name__ == '__main__':
    waypoint_specified = True
    waypoint_bc = False
    rospy.init_node('Trajectory_test', anonymous=False, log_level=rospy.DEBUG)
    path = [[-7,  4,  0.08 ], [-7,   4,  2.0]]
    Minimum_snap_trajetory(path, waypoint_specified, waypoint_bc)
    #r = rospy.Rate(15)
    
    try: 
        while not rospy.is_shutdown(): 
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass