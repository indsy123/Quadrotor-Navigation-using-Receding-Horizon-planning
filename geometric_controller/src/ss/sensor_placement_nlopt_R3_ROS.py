#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 11 13:59:38 2019
code to publish the target location for individual robots using nonlinear optimization
@author: ind
"""

import numpy as np 
import matplotlib.pyplot as plt
#from scipy import *
#from matplotlib import cm
#import os
#from matplotlib.patches import Polygon 
#from mpl_toolkits.mplot3d import Axes3D
#import time 
import nlopt, rospy, time
from nav_msgs.msg import Odometry
import message_filters
from scipy.optimize import linear_sum_assignment
from geometric_controller.msg import target_positions
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PointStamped

class optimize_edgelength(object):
    def __init__(self, freq):
        self.d1 = 1.5 # safety distance between two quads
        self.d2 = 0.5 # safety distance with the target
        self.d3 = 1.5 # max distance with the target
        self.no_of_robots  = 3
        self.xtarget = np.array([0, 6.5, 0.5]) # (x,y) values of target
        self.xr1 = np.array([-8, -8, 0.5])
        self.xr2 = np.array([8, -8, 0.5])
        self.xr3 = np.array([0, 0, 0.5])
        T = 100; self.dt = 1.0/freq
        self.v = 0.5
        self.counter = 0
        self.tt = time.time()
        self.time = np.linspace(0,T,(T/self.dt)+1)
        self.robots = np.array([self.xr1, self.xr2, self.xr3])
        #print 'i m here1'
        self.pub = rospy.Publisher('/target_positions_for_quads', target_positions, queue_size = 500, tcp_nodelay = True)
        rospy.Subscriber('/turtlebot/ground_truth/state', Odometry, self.target_callback, queue_size = 20, tcp_nodelay = True)
        #rospy.Subscriber('/gazebo/link_states', LinkStates, self.target_callback, tcp_nodelay = True)
        #rospy.Subscriber('/goal_to_chase', PointStamped, self.target_callback2, tcp_nodelay = True)
        rospy.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, self.callback1, queue_size = 10, tcp_nodelay = True)
        rospy.Subscriber('/firefly2/odometry_sensor1/odometry', Odometry, self.callback2, queue_size = 10, tcp_nodelay = True)
        rospy.Subscriber('/firefly3/odometry_sensor1/odometry', Odometry, self.callback3, queue_size = 10, tcp_nodelay = True)
        
        #try: 
        #    r1 = message_filters.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
        #    r2 = message_filters.Subscriber('/firefly2/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
        #    r3 = message_filters.Subscriber('/firefly3/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
        #    r4 = rospy.Subscriber('/gazebo/link_states', LinkStates, tcp_nodelay = True)
        #    #rt = message_filters.Subscriber('/turtlebot/ground_truth/state', Odometry, tcp_nodelay = True)
        #    ts = message_filters.TimeSynchronizer([r1, r2, r3], 10)
        #    ts.registerCallback(self.robot_callback)   
        #except: 
        #    print 'there is a problem subscribing to one of the quad odometry topics'

    def target_callback(self, r4): 
        """gets the position of the target"""
        #rospy.loginfo('target_position_callback')
        self.xtarget = np.array([r4.pose.pose.position.x, r4.pose.pose.position.y, r4.pose.pose.position.z + 0.5])
        #self.xtarget = np.array([r4.pose[40].position.x, r4.pose[40].position.y, r4.pose[40].position.z+0.5])
        #rospy.loginfo('the target location is %f, %f, %f:', self.xtarget[0], self.xtarget[1], self.xtarget[2])

    def target_callback2(self, r4): 
        """gets the position of the target"""
        #rospy.loginfo('target_position_callback')
        self.xtarget = np.array([r4.point.x, r4.point.y, r4.point.z+0.5])
        #rospy.loginfo('the target location is %f, %f, %f:', self.xtarget[0], self.xtarget[1], self.xtarget[2])    
    def callback1(self, r1): 
        """gets the position of the robot1"""
        #rospy.loginfo('r1_callback')
        self.xr1 = np.array([r1.pose.pose.position.x, r1.pose.pose.position.y, r1.pose.pose.position.z])
        #rospy.loginfo('the r1 location is %f, %f, %f:', self.xr1[0], self.xr1[1], self.xr1[2])

    def callback2(self, r2): 
        """gets the position of the robot2"""
        #rospy.loginfo('r2_callback')
        self.xr2 = np.array([r2.pose.pose.position.x, r2.pose.pose.position.y, r2.pose.pose.position.z])
        #rospy.loginfo('the r2 location is %f, %f, %f:', self.xr2[0], self.xr2[1], self.xr2[2])
        
    def callback3(self, r3): 
        """gets the position of the robot3"""
        #rospy.loginfo('r3_callback')
        self.xr3 = np.array([r3.pose.pose.position.x, r3.pose.pose.position.y, r3.pose.pose.position.z])
        #rospy.loginfo('the r3 location is %f, %f, %f:', self.xr3[0], self.xr3[1], self.xr3[2])
        
    def robot_callback(self, r1, r2, r3): 
        """this callback publishes the positions of the robots to use in hungarian algorithm"""
        #rospy.loginfo('robot_callback')
        #self.xtarget = np.array([rt.pose.pose.position.x, rt.pose.pose.position.y, rt.pose.pose.position.z + 1.0])
        
        xr1 = np.array([r1.pose.pose.position.x, r1.pose.pose.position.y, r1.pose.pose.position.z])
        xr2 = np.array([r2.pose.pose.position.x, r2.pose.pose.position.y, r2.pose.pose.position.z])
        xr3 = np.array([r3.pose.pose.position.x, r3.pose.pose.position.y, r3.pose.pose.position.z])
        self.robots = np.array([xr1, xr2, xr3])

        
    def myfunc(self, x, grad):
        w = np.array([0.1, 0.1, 0.1]); w = w/np.linalg.norm(w)
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 4*w[0]*np.linalg.norm(xr[0]-self.xtarget)**2 * (xr[0][0]-self.xtarget[0])
            grad[1] = 4*w[0]*np.linalg.norm(xr[0]-self.xtarget)**2 * (xr[0][1]-self.xtarget[1])
            grad[2] = 4*w[0]*np.linalg.norm(xr[0]-self.xtarget)**2 * (xr[0][2]-self.xtarget[2])
            grad[3] = 4*w[1]*np.linalg.norm(xr[1]-self.xtarget)**2 * (xr[1][0]-self.xtarget[0])
            grad[4] = 4*w[1]*np.linalg.norm(xr[1]-self.xtarget)**2 * (xr[1][1]-self.xtarget[1])
            grad[5] = 4*w[1]*np.linalg.norm(xr[1]-self.xtarget)**2 * (xr[1][2]-self.xtarget[2])
            grad[6] = 4*w[2]*np.linalg.norm(xr[2]-self.xtarget)**2 * (xr[2][0]-self.xtarget[0])
            grad[7] = 4*w[2]*np.linalg.norm(xr[2]-self.xtarget)**2 * (xr[2][1]-self.xtarget[1])
            grad[8] = 4*w[2]*np.linalg.norm(xr[2]-self.xtarget)**2 * (xr[2][2]-self.xtarget[2])
        return w[0]*np.linalg.norm(xr[0]-self.xtarget)**4 + w[1]*np.linalg.norm(xr[1]-self.xtarget)**4 + \
        w[2]*np.linalg.norm(xr[2]-self.xtarget)**4
    
    def myconstraint1(self, x, grad): #||x1-x2||>=d1
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = -(xr[0][0]-xr[1][0])/np.linalg.norm(xr[0]-xr[1])
            grad[1] = -(xr[0][1]-xr[1][1])/np.linalg.norm(xr[0]-xr[1])
            grad[2] = -(xr[0][2]-xr[1][2])/np.linalg.norm(xr[0]-xr[1])
            grad[3] = -grad[0]; grad[4] = -grad[1]; grad[5] = -grad[2]
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0        
        return self.d1 - np.linalg.norm(xr[0]-xr[1])
    
    def myconstraint2(self, x, grad):#||x2-x3||>=d1
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = -(xr[1][0]-xr[2][0])/np.linalg.norm(xr[1]-xr[2]) 
            grad[4] = -(xr[1][1]-xr[2][1])/np.linalg.norm(xr[1]-xr[2])
            grad[5] = -(xr[1][2]-xr[2][2])/np.linalg.norm(xr[1]-xr[2])
            grad[6] = -grad[3]; grad[7] = -grad[4]; grad[8] = -grad[5]
        return self.d1 - np.linalg.norm(xr[1]-xr[2])
       
    def myconstraint3(self, x, grad):#||x3-x1||>=d1
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = (xr[2][0]-xr[0][0])/np.linalg.norm(xr[2]-xr[0])
            grad[1] = (xr[2][1]-xr[0][1])/np.linalg.norm(xr[2]-xr[0])
            grad[2] = (xr[2][2]-xr[0][2])/np.linalg.norm(xr[2]-xr[0])
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = -grad[0]; grad[7] = -grad[1]; grad[8] = -grad[2]
        return self.d1 - np.linalg.norm(xr[2]-xr[0])
        
        
    def myconstraint4(self, x, grad):#||x1-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = -(xr[0][0]-self.xtarget[0])/np.linalg.norm(xr[0]-self.xtarget)
            grad[1] = -(xr[0][1]-self.xtarget[1])/np.linalg.norm(xr[0]-self.xtarget)
            grad[2] = -(xr[0][2]-self.xtarget[2])/np.linalg.norm(xr[0]-self.xtarget)
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
        return self.d2 - np.linalg.norm(xr[0]-self.xtarget)
        
    def myconstraint5(self, x, grad):#||x2-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = -(xr[1][0]-self.xtarget[0])/np.linalg.norm(xr[1]-self.xtarget)
            grad[4] = -(xr[1][1]-self.xtarget[1])/np.linalg.norm(xr[1]-self.xtarget)
            grad[5] = -(xr[1][2]-self.xtarget[2])/np.linalg.norm(xr[1]-self.xtarget)
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
        return self.d2 - np.linalg.norm(xr[1]-self.xtarget)
    
    def myconstraint6(self, x, grad):#||x3-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = -(xr[2][0]-self.xtarget[0])/np.linalg.norm(xr[2]-self.xtarget)
            grad[7] = -(xr[2][1]-self.xtarget[1])/np.linalg.norm(xr[2]-self.xtarget)
            grad[8] = -(xr[2][2]-self.xtarget[2])/np.linalg.norm(xr[2]-self.xtarget)
        return self.d2 - np.linalg.norm(xr[2]-self.xtarget)
        
    def myconstraint7(self, x, grad):#||x3-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = -1.0
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
        return self.xtarget[2]- xr[0][2] + 0.5

    def myconstraint8(self, x, grad):#||x3-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = -1.0
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
        return self.xtarget[2]- xr[1][2] + 0.5

    def myconstraint9(self, x, grad):#||x3-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = -1.0
        return self.xtarget[2]- xr[2][2] + 0.5


    def myconstraint10(self, x, grad):#||x1-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = (xr[0][0]-self.xtarget[0])/np.linalg.norm(xr[0]-self.xtarget)
            grad[1] = (xr[0][1]-self.xtarget[1])/np.linalg.norm(xr[0]-self.xtarget)
            grad[2] = (xr[0][2]-self.xtarget[2])/np.linalg.norm(xr[0]-self.xtarget)
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
        return np.linalg.norm(xr[0]-self.xtarget) - self.d3 
        
    def myconstraint11(self, x, grad):#||x2-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = (xr[1][0]-self.xtarget[0])/np.linalg.norm(xr[1]-self.xtarget)
            grad[4] = (xr[1][1]-self.xtarget[1])/np.linalg.norm(xr[1]-self.xtarget)
            grad[5] = (xr[1][2]-self.xtarget[2])/np.linalg.norm(xr[1]-self.xtarget)
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
        return np.linalg.norm(xr[1]-self.xtarget) - self.d3
    
    def myconstraint12(self, x, grad):#||x3-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        if grad.size > 0:
            grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = (xr[2][0]-self.xtarget[0])/np.linalg.norm(xr[2]-self.xtarget)
            grad[7] = (xr[2][1]-self.xtarget[1])/np.linalg.norm(xr[2]-self.xtarget)
            grad[8] = (xr[2][2]-self.xtarget[2])/np.linalg.norm(xr[2]-self.xtarget)
        return np.linalg.norm(xr[2]-self.xtarget) - self.d3
        
    def myconstraint22(self, x, grad):#||x3-xt||>=d2
        xr = [np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])]
        #aa = np.sqrt((xr[0][0]-self.xt[0])**2 + (xr[0][1]-self.xt[1])**2 + (xr[0][2]-self.xt[2])**2)
        if grad.size > 0:
            grad[0] = 2*(xr[0][0]-self.xt[0]); grad[1] = 2*(xr[0][1]-self.xt[1]); grad[2] = -2*(xr[0][2]-self.xt[2])
            grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0
            grad[6] = 0.0; grad[7] = 0.0; grad[8] = 0.0
            
        return (xr[0][0]-self.xt[0])**2 + (xr[0][1]-self.xt[1])**2 - (xr[0][2]-self.xt[2])**2
        
        
        
    def plot(self, xt, x1, x2, x3): 
        xx1 = []; yy1 = []; zz1 = []
        xx2 = []; yy2 = []; zz2 = []
        xx3 = []; yy3 = []; zz3 = []
        #xt.append(self.xt)
        xt = zip(*xt)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(len(x1)): 
            xx1.append(x1[i][0]); yy1.append(x1[i][1]); zz1.append(x1[i][2])
            xx2.append(x2[i][0]); yy2.append(x2[i][1]); zz2.append(x2[i][2])
            xx3.append(x3[i][0]); yy3.append(x3[i][1]); zz3.append(x3[i][2])


        ax.scatter(xt[0], xt[1], marker = '*', color = 'r')
        ax.scatter(xx1, yy1, zz1, marker = 'o', color='b')
        ax.scatter(xx2, yy2, zz2, marker = 'o', color='g')
        ax.scatter(xx3, yy3, zz3, marker = 'o', color='m')

        #ax.axes.set_xlim3d(-2, 2)
        #ax.axes.set_ylim3d(-2, 2)
        #ax.axes.set_zlim3d(-2, 2)
        ax.set_xlabel('$X$', fontsize=20)
        ax.set_ylabel('$Y$', fontsize=20)
        ax.set_zlabel('$Z$', fontsize=20)
        ax.view_init(30,30)
        #plt.tight_layout()
        plt.show()

    def hungarian_algorithm(self): 
        """impliment the hungarian algorithm"""
        # make the cost matrix
        c = np.zeros((3,3))
        for i in range(self.no_of_robots): 
            for j in range(self.no_of_robots): 
                c[i,j] = np.linalg.norm(self.robots[i]-self.xtp[j])
       
        row_index, col_index = linear_sum_assignment(c)
        return row_index, col_index
           

            
    def optimize(self): 
        """determines the points using optimization"""
        rospy.loginfo('1')
        self.robots = np.array([self.xr1, self.xr2, self.xr3])
        a1 = np.linalg.norm(self.xr1-self.xr2); a2 = np.linalg.norm(self.xr1-self.xr3); a3 = np.linalg.norm(self.xr3-self.xr2)
        rospy.loginfo('distance between quads are: %f, %f, %f', a1, a2, a3)
        rospy.loginfo('target is at: %f, %f, %f', self.xtarget[0], self.xtarget[1], self.xtarget[2])
        msg = target_positions()
        t = time.time() - self.tt
        rx = 8; ry = 6.5; secs = 2
        x = rx*np.sin((t/secs/200)*180/np.pi); y = ry*np.cos((t/secs/200)*180/np.pi)
        #rospy.loginfo("target is at %f, %f, %f", self.xtarget[0], self.xtarget[1], self.xtarget[2])
        #self.xtarget = np.array([x, y, 0.5]) 
        #x1 = []; x2 = []; x3 = []; xt = []
        #for i in self.time:
        #self.xt = self.xt + np.array([self.v*self.dt, self.v*self.dt, 0])
        #xt.append(list(self.xt))
        
        #pub = rospy.Publisher('/target_positions_for_quads', target_positions, queue_size = 1, tcp_nodelay = True)
        #msg = target_positions()
        #while not rospy.is_shutdown(): 
        # could also use LD_SLSQP    
        
        opt = nlopt.opt(nlopt.LD_MMA, 9)
        #opt.set_lower_bounds([-float('inf'), -float('inf'), -float('inf'), -float('inf'), -float('inf'), -float('inf')])
        #opt.set_upper_bounds([100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
        #opt.set_lower_bounds([-100.0, -100.0, 0.0, -100.0, -100.0, 0.0, -100.0, -100.0, 0.0])
        rospy.loginfo('3')
        #opt.set_upper_bounds([100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
        #opt.set_lower_bounds([-100.0, -100.0, 0.0, -100.0, -100.0, 0.0, -100.0, -100.0, 0.0])
        
        opt.set_upper_bounds([100.0+self.xtarget[0], 100.0+self.xtarget[1], 100.0+self.xtarget[2], 100.0+self.xtarget[0], \
        100.0+self.xtarget[1], 100.0+self.xtarget[2], 100.0+self.xtarget[0], 100.0+self.xtarget[1], 100.0+self.xtarget[2]])
        opt.set_lower_bounds([-100.0-self.xtarget[0], -100.0-self.xtarget[1], 0.0, -100.0-self.xtarget[0], \
        -100.0-self.xtarget[1], 0.0, -100.0-self.xtarget[0], -100.0-self.xtarget[1], 0.0])
        opt.set_min_objective(self.myfunc)
        rospy.loginfo('4')
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint1(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint2(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint3(x,grad), 1e-8)
        
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint4(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint5(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint6(x,grad), 1e-8)
        
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint7(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint8(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint9(x,grad), 1e-8) 
        
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint10(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint11(x,grad), 1e-8)
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint12(x,grad), 1e-8)  
        
        opt.add_inequality_constraint(lambda x,grad: self.myconstraint22(x,grad), 1e-8)
        rospy.loginfo('4')
        opt.set_xtol_rel(1e-6); #opt.maxeval=100000000000
        if self.counter == 0: 
            rospy.loginfo('5')

            x = opt.optimize([self.xtarget[0]+0.9, self.xtarget[1]+0.6, self.xtarget[2]+0.4, self.xtarget[0]+0.8\
            , self.xtarget[1]-0.6, self.xtarget[2]+0.2, self.xtarget[0]-0.5, self.xtarget[1]+0.2, self.xtarget[2]+0.50])

            #x = opt.optimize([self.xtarget[0], self.xtarget[1], self.xtarget[2], self.xtarget[0]\
            #, self.xtarget[1], self.xtarget[2], self.xtarget[0], self.xtarget[1], self.xtarget[2]])                
            #x = opt.optimize([0+0.5, 6.5+0.6, 0.5+0.4, 0+0.8, 6.5-0.4, 0.5+0.2, 0-0.3, 6.5+0.2, 0.5+0.10])
            rospy.loginfo('6')
            self.xtp = np.array([np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])])
            
            row_index, col_index = self.hungarian_algorithm()            
            
            msg.robot1.x = self.xtp[row_index[0]][0]; msg.robot1.y = self.xtp[row_index[0]][1]; msg.robot1.z = self.xtp[row_index[0]][2]
            msg.robot2.x = self.xtp[row_index[1]][0]; msg.robot2.y = self.xtp[row_index[1]][1]; msg.robot2.z = self.xtp[row_index[1]][2]
            msg.robot3.x = self.xtp[row_index[2]][0]; msg.robot3.y = self.xtp[row_index[2]][1]; msg.robot3.z = self.xtp[row_index[2]][2]
            self.pub.publish(msg)
            self.counter += 1
            rospy.loginfo('6')
        
        else:
            rospy.loginfo('7')
            x = opt.optimize([self.xtarget[0]+0.5, self.xtarget[1]+0.6, self.xtarget[2]+0.4, self.xtarget[0]+0.8\
            , self.xtarget[1]-0.6, self.xtarget[2]+0.2, self.xtarget[0]-0.5, self.xtarget[1]+0.2, self.xtarget[2]+0.50])
            #x = opt.optimize([self.xtp[0][0], self.xtp[0][1], self.xtp[0][2], self.xtp[1][0], self.xtp[1][1], \
            #self.xtp[1][2], self.xtp[2][0], self.xtp[2][1], self.xtp[2][2]])

            self.xtp = np.array([np.array([x[0], x[1], x[2]]), np.array([x[3], x[4], x[5]]), np.array([x[6], x[7], x[8]])])

            row_index, col_index = self.hungarian_algorithm()
            
            msg.robot1.x = self.xtp[row_index[0]][0]; msg.robot1.y = self.xtp[row_index[0]][1]; msg.robot1.z = self.xtp[row_index[0]][2]
            msg.robot2.x = self.xtp[row_index[1]][0]; msg.robot2.y = self.xtp[row_index[1]][1]; msg.robot2.z = self.xtp[row_index[1]][2]
            msg.robot3.x = self.xtp[row_index[2]][0]; msg.robot3.y = self.xtp[row_index[2]][1]; msg.robot3.z = self.xtp[row_index[2]][2]
            self.pub.publish(msg)
            self.counter += 1
            rospy.loginfo('8')
        #rospy.loginfo('i m here')
        b1 = np.linalg.norm(self.xtp[0]-self.xtp[1]); b2 = np.linalg.norm(self.xtp[1]-self.xtp[2])
        b3 = np.linalg.norm(self.xtp[0]-self.xtp[2])
        rospy.loginfo('actual distance: %f, %f, %f', a1, a2, a3)
        rospy.loginfo('output distance: %f, %f, %f', b1, b2, b3)


if __name__ == '__main__': 
    rospy.init_node('publish_targets', anonymous=True, log_level=rospy.DEBUG)
    #self.pub = rospy.Publisher('/target_positions_for_quads', target_positions, queue_size = 1, tcp_nodelay = True)
    freq = 10.0
    r = rospy.Rate(freq)
    f = optimize_edgelength(freq)
    try:
        while not rospy.is_shutdown():
            f.optimize()
            r.sleep()
    except rospy.ROSInterruptException(): pass