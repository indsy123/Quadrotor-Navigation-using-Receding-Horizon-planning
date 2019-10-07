#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 13:59:38 2019
code to publish the target location for individual robots using qcp
@author: indrajeet
"""
import numpy as np 
from gurobipy import *
import time, scipy, operator
from functools import reduce
from scipy.linalg import block_diag
from gurobipy import *
from multiprocessing import Process, Manager, Pool
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
from geometric_controller.msg import target_positions_5quad
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry, Path
import rospy, message_filters
from scipy.optimize import linear_sum_assignment
from geometry_msgs.msg import PoseStamped
from itertools import combinations


class sensor_placement_optimization_qcp(object):
    def __init__(self, freq): 
        """initializes variables"""
        self.path1 = Path(); self.path2 = Path(); self.path3 = Path()
        self.path4 = Path(); self.path5 = Path()
        #self.freq = freq
        self.xt = np.array([0, 6.5, 1])
        self.no_of_robots = 5
        self.dimension = 3 # dimension: 2D or 3D
        self.drr = 1.5 # minimum distance between robots
        self.drt_min = 0.25 # minimum distance with the target
        self.drt_max = 1.25 # max distance with the target
        self.heigt_difference = 0.5
        self.dt = 1.0/freq
        self.T = 1
        self.counter = 0
        self.time = time.time()
        open("target_position.txt", "w").close()
        open("robot1_position.txt", "w").close()
        open("robot2_position.txt", "w").close()
        open("robot3_position.txt", "w").close()
        open("robot4_position.txt", "w").close()
        open("robot5_position.txt", "w").close()
        self.robots = np.array([np.array([-9.5,-7,3]), np.array([9,-9,3]), np.array([9,10,3]), np.array([0,-9,3]), np.array([9,0,3])])
        self.time_points = np.linspace(0,self.T,(self.T/self.dt)+1)
        self.pub = rospy.Publisher('/target_positions_for_quads', target_positions_5quad, queue_size = 50, tcp_nodelay = True)
        self.pub1 = rospy.Publisher('/path1', Path, queue_size = 50, tcp_nodelay = True)
        self.pub2 = rospy.Publisher('/path2', Path, queue_size = 50, tcp_nodelay = True)
        self.pub3 = rospy.Publisher('/path3', Path, queue_size = 50, tcp_nodelay = True)
        self.pub4 = rospy.Publisher('/path4', Path, queue_size = 50, tcp_nodelay = True)
        self.pub5 = rospy.Publisher('/path5', Path, queue_size = 50, tcp_nodelay = True)
        
        rospy.Subscriber('/turtlebot/ground_truth/state', Odometry, self.target_callback, tcp_nodelay = True)
            
        try: 
            r1 = message_filters.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
            r2 = message_filters.Subscriber('/firefly2/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
            r3 = message_filters.Subscriber('/firefly3/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
            r4 = message_filters.Subscriber('/firefly4/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
            r5 = message_filters.Subscriber('/firefly5/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([r1, r2, r3, r4, r5], 10)
            ts.registerCallback(self.robot_callback)   
        except: 
            print 'there is a problem subscribing to one of the quad odometry topics'

    def target_callback(self, r6): 
        """gets the position of the target"""
        self.xt = np.array([r6.pose.pose.position.x, r6.pose.pose.position.y, r6.pose.pose.position.z+0.5])        
        f1 = open('target_position.txt', 'a')
        f1.write("%s, %s, %s, %s\n" % (time.time(), self.xt[0], self.xt[1], self.xt[2]))

   
    def robot_callback(self, r1, r2, r3, r4, r5): 
        """this callback publishes the positions of the robots to use in hungarian algorithm"""

        global path1, path2, path3, path4, path5
        
        pose1 = PoseStamped(); pose2 = PoseStamped(); pose3 = PoseStamped()
        pose4 = PoseStamped(); pose5 = PoseStamped()
        
        self.path1.header = r1.header; pose1.header = r1.header
        pose1.pose = r1.pose.pose; self.path1.poses.append(pose1); self.pub1.publish(self.path1)

        self.path2.header = r2.header; pose2.header = r2.header
        pose2.pose = r2.pose.pose; self.path2.poses.append(pose2); self.pub2.publish(self.path2)

        self.path3.header = r3.header; pose3.header = r3.header
        pose3.pose = r3.pose.pose; self.path3.poses.append(pose3); self.pub3.publish(self.path3)

        self.path4.header = r4.header; pose4.header = r4.header
        pose4.pose = r4.pose.pose; self.path4.poses.append(pose4); self.pub4.publish(self.path4)

        self.path5.header = r5.header; pose5.header = r5.header
        pose5.pose = r5.pose.pose; self.path5.poses.append(pose5); self.pub5.publish(self.path5)
        
        xr1 = np.array([r1.pose.pose.position.x, r1.pose.pose.position.y, r1.pose.pose.position.z])
        xr2 = np.array([r2.pose.pose.position.x, r2.pose.pose.position.y, r2.pose.pose.position.z])
        xr3 = np.array([r3.pose.pose.position.x, r3.pose.pose.position.y, r3.pose.pose.position.z])
        xr4 = np.array([r4.pose.pose.position.x, r4.pose.pose.position.y, r4.pose.pose.position.z])
        xr5 = np.array([r5.pose.pose.position.x, r5.pose.pose.position.y, r5.pose.pose.position.z])
        self.robots = np.array([xr1, xr2, xr3, xr4, xr5])
        #rospy.loginfo(' the time is %f:', time.time()-self.time)
        f2 = open('robot1_position.txt', 'a')
        f2.write("%s, %s, %s, %s\n" % (time.time(), xr1[0], xr1[1], xr1[2]))
        f3 = open('robot2_position.txt', 'a')
        f3.write("%s, %s, %s, %s\n" % (time.time(), xr2[0], xr2[1], xr2[2]))
        f4 = open('robot3_position.txt', 'a')
        f4.write("%s, %s, %s, %s\n" % (time.time(), xr3[0], xr3[1], xr3[2]))
        f5 = open('robot4_position.txt', 'a')
        f5.write("%s, %s, %s, %s\n" % (time.time(), xr4[0], xr4[1], xr4[2]))
        f6 = open('robot5_position.txt', 'a')
        f6.write("%s, %s, %s, %s\n" % (time.time(), xr5[0], xr5[1], xr5[2]))

    def hungarian_algorithm(self): 
        """impliment the hungarian algorithm"""
        # make the cost matrix
        c = np.zeros((5,5))
        for i in range(self.no_of_robots): 
            for j in range(self.no_of_robots): 
                c[i,j] = np.linalg.norm(self.robots[i]-self.xtp[j])**2
       
        row_index, col_index = linear_sum_assignment(c)
        return row_index, col_index

    def optimize(self):
        """ function to construct the trajectory using parallel computation"""
        n = self.no_of_robots
        msg = target_positions_5quad()
        m = Model("qcp")            
        x = m.addVars(n, lb = -15, ub = 15, vtype=GRB.CONTINUOUS, name="x")
        y = m.addVars(n, lb = -15, ub = 15, vtype=GRB.CONTINUOUS, name="y")
        z = m.addVars(n, lb = 0, ub = 15, vtype=GRB.CONTINUOUS, name="z")
        #p = m.addVars(n, vtype=GRB.CONTINUOUS, name="p")

        Q = np.eye(n,n)
        m.update()
        w1 = 0.8; w2 = 0.1; w3 = 0.1; w4 = 0.1; w5 = 0.1; w = [w1, w2, w3, w4, w5]
        obj1 = quicksum(w[i]*(x[i]-self.xt[0]) * quicksum(Q[i][j] * (x[j]-self.xt[0]) for j in range(n)) for i in range(n))
        obj2 = quicksum(w[i]*(y[i]-self.xt[1]) * quicksum(Q[i][j] * (y[j]-self.xt[1]) for j in range(n)) for i in range(n))
        obj3 = quicksum(w[i]*(z[i]-self.xt[2]) * quicksum(Q[i][j] * (z[j]-self.xt[2]) for j in range(n)) for i in range(n))
        obj = obj1 + obj2 + obj3

        # since greater than and equal to constraints are not convex, we need to approximate them
        # see the unsolved problem 8.27 from Convex Optimization by S Boyd
        #choose a random initial position for the 3 robots
        if self.counter == 0: 
            xr_initial = [self.xt + np.array([np.random.uniform(0.5,1), np.random.uniform(-1,1), \
            np.random.uniform(0,1)]) for i in range(n)]
            #xr_initial.append(xr_initial[0]) # append first element to find a's easily
        else: 
            xr_initial = [self.x1, self.x2, self.x3, self.x4, self.x5]
            #xr_initial.append(xr_initial[0])
        
        b = [(xr_initial[i]-self.xt)/np.linalg.norm(xr_initial[i]-self.xt) for i in range(n)]
        m.addConstr(b[0][0]*(x[0]-self.xt[0]) + b[0][1]*(y[0]-self.xt[1]) + b[0][2]*(z[0]-self.xt[2]) >= self.drt_min)
        m.addConstr(b[1][0]*(x[1]-self.xt[0]) + b[1][1]*(y[1]-self.xt[1]) + b[1][2]*(z[1]-self.xt[2]) >= self.drt_min)
        m.addConstr(b[2][0]*(x[2]-self.xt[0]) + b[2][1]*(y[2]-self.xt[1]) + b[2][2]*(z[2]-self.xt[2]) >= self.drt_min)
        m.addConstr(b[3][0]*(x[3]-self.xt[0]) + b[3][1]*(y[3]-self.xt[1]) + b[3][2]*(z[3]-self.xt[2]) >= self.drt_min)  
        m.addConstr(b[4][0]*(x[4]-self.xt[0]) + b[4][1]*(y[4]-self.xt[1]) + b[4][2]*(z[4]-self.xt[2]) >= self.drt_min)


        if self.counter == 0.0: 
            comb = combinations(range(n), 2)
            for k in comb:
                #print k, k[0], k[1]
                s1 = self.xt + np.array([np.random.uniform(-1,1), np.random.uniform(-1,1), np.random.uniform(0,1)])
                s2 = self.xt + np.array([np.random.uniform(-1,1), np.random.uniform(-1,1), np.random.uniform(0,1)])
                a = (s1-s2)/np.linalg.norm(s1-s2)

                m.addConstr(a[0]*(x[k[0]]-x[k[1]]) + a[1]*(y[k[0]]-y[k[1]]) + a[2]*(z[k[0]]-z[k[1]]) >= self.drr)
        else: 
            comb = combinations(range(n), 2)
            for k in comb: 
   
                s1 = self.xtp[k[0]]; s2 = self.xtp[k[1]]
                a = (s1-s2)/np.linalg.norm(s1-s2)
                m.addConstr(a[0]*(x[k[0]]-x[k[1]]) + a[1]*(y[k[0]]-y[k[1]]) + a[2]*(z[k[0]]-z[k[1]]) >= self.drr)  
                
                
                
        
     
        #a = [(xr_initial[i]-xr_initial[i+1])/np.linalg.norm(xr_initial[i]-xr_initial[i+1]) for i in range(len(xr_initial)-1)]
        #b = [(xr_initial[i]-self.xt)/np.linalg.norm(xr_initial[i]-self.xt) for i in range(len(xr_initial)-1)]
        
        #m.addConstr(a[0][0]*(x[0]-x[1]) + a[0][1]*(y[0]-y[1]) + a[0][2]*(z[0]-z[1])>= self.drr)
        #m.addConstr(a[1][0]*(x[1]-x[2]) + a[1][1]*(y[1]-y[2]) + a[1][2]*(z[1]-z[2]) >= self.drr)
        #m.addConstr(a[2][0]*(x[2]-x[0]) + a[2][1]*(y[2]-y[0]) + a[2][2]*(z[2]-z[0])>= self.drr)
        
        
        #m.addConstr(a[3][0]*(x[1]-x[2]) + a[3][1]*(y[1]-y[2]) + a[3][2]*(z[1]-z[2]) >= self.drr)
        #m.addConstr(a[4][0]*(x[2]-x[0]) + a[4][1]*(y[2]-y[0]) + a[4][2]*(z[2]-z[0])>= self.drr)
 
        #m.addConstr(b[0][0]*(x[0]-self.xt[0]) + b[0][1]*(y[0]-self.xt[1]) + b[0][2]*(z[0]-self.xt[2])>= self.drt_min)
        #m.addConstr(b[1][0]*(x[1]-self.xt[0]) + b[1][1]*(y[1]-self.xt[1]) + b[1][2]*(z[1]-self.xt[2])>= self.drt_min)
        #m.addConstr(b[2][0]*(x[2]-self.xt[0]) + b[2][1]*(y[2]-self.xt[1]) + b[2][2]*(z[2]-self.xt[2])>= self.drt_min)

        m.addConstr(z[0] >= self.xt[2] + self.heigt_difference)
        m.addConstr(z[1] >= self.xt[2] + self.heigt_difference)
        m.addConstr(z[2] >= self.xt[2] + self.heigt_difference)
        m.addConstr(z[3] >= self.xt[2] + self.heigt_difference)
        m.addConstr(z[4] >= self.xt[2] + self.heigt_difference)
        
        #m.addConstr((z[0]-self.xt[2])*(z[0]-self.xt[2]) <= p[0])
        #m.addConstr((x[0]-self.xt[0])*(x[0]-self.xt[0]) + (y[0]-self.xt[1])*(y[0]-self.xt[1]) <= p[0])
         
        m.setObjective(obj, GRB.MINIMIZE)
        #m.write('model.lp')
        m.setParam('OutputFlag', 0) 
        m.setParam('PSDtol', 1e-3) 

        m.optimize()
        
        #runtime = m.Runtime

        #if m.status == 2 or m.status == 13: 
        #    self.x1 = np.array([x[0].X, y[0].X, z[0].X])
        #    self.x2 = np.array([x[1].X, y[1].X, z[1].X])
        #    self.x3 = np.array([x[2].X, y[2].X, z[2].X])
        #    self.xtp = np.array([self.x1, self.x2, self.x3])
        #else: 
        #    self.x1 = self.x1 + np.array([0.01, 0.01, 0.01])
        #    self.x2 = self.x2 + np.array([0.01, 0.01, 0.01])
        #    self.x3 = self.x3 + np.array([0.01, 0.01, 0.01])
        #    self.xtp = np.array([self.x1, self.x2, self.x3])   
            
            
            
        if m.status == 2 or m.status == 13: 
            #opt_subopt += 1
            self.x1 = np.array([x[0].X, y[0].X, z[0].X])
            self.x2 = np.array([x[1].X, y[1].X, z[1].X])
            self.x3 = np.array([x[2].X, y[2].X, z[2].X])
            self.x4 = np.array([x[3].X, y[3].X, z[3].X])
            self.x5 = np.array([x[4].X, y[4].X, z[4].X])
            self.xtp = np.array([self.x1, self.x2, self.x3, self.x4, self.x5])
        else: 
            #nosol += 1
            self.x1 = self.x1 + np.array([0.01, 0.01, 0.01])
            self.x2 = self.x2 + np.array([0.01, 0.01, 0.01])
            self.x3 = self.x3 + np.array([0.01, 0.01, 0.01])
            self.x4 = self.x4 + np.array([0.01, 0.01, 0.01])
            self.x5 = self.x5 + np.array([0.01, 0.01, 0.01])
            self.xtp = np.array([self.x1, self.x2, self.x3, self.x4, self.x5])

        row_index, col_index = self.hungarian_algorithm()        
       
        msg.robot1.x = self.xtp[row_index[0]][0]; msg.robot1.y = self.xtp[row_index[0]][1]; msg.robot1.z = self.xtp[row_index[0]][2]
        msg.robot2.x = self.xtp[row_index[1]][0]; msg.robot2.y = self.xtp[row_index[1]][1]; msg.robot2.z = self.xtp[row_index[1]][2]
        msg.robot3.x = self.xtp[row_index[2]][0]; msg.robot3.y = self.xtp[row_index[2]][1]; msg.robot3.z = self.xtp[row_index[2]][2]
        msg.robot4.x = self.xtp[row_index[3]][0]; msg.robot4.y = self.xtp[row_index[3]][1]; msg.robot4.z = self.xtp[row_index[3]][2]
        msg.robot5.x = self.xtp[row_index[4]][0]; msg.robot5.y = self.xtp[row_index[4]][1]; msg.robot5.z = self.xtp[row_index[4]][2]
        self.pub.publish(msg)
        self.counter += 1



if __name__ == '__main__': 
    rospy.init_node('publish_targets_using_qcp', anonymous=True, log_level=rospy.DEBUG)
    freq = 100.0
    r = rospy.Rate(freq)
    f = sensor_placement_optimization_qcp(freq)
    try:
        while not rospy.is_shutdown():
            f.optimize()
            r.sleep()
    except rospy.ROSInterruptException(): pass
