#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 08:57:11 2018
Revised by indrajeet to include any number of obstacles specified by user
@author: indrajeet
"""
import numpy as np 
import matplotlib.pyplot as plt
import time, rospy
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

from numpy import sin as sin
from numpy import cos as cos
from numpy import pi as pi 
from numpy.linalg import norm as nrm
from numpy import sqrt as sq
from numpy import power as pwr
from operator import mul, add
#from numpy.random import uniform as uni

import scipy.interpolate
#from multiprocessing import Process, Pool, Manager
#from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

#import message_filters
#from gazebo_msgs.msg import LinkStates
from geometric_controller.msg import velocities_from_navigation_function, target_positions
#from isy_geometric_controller.msg import Desired_Trajectory
from nav_msgs.msg import Odometry
from scipy.interpolate import splrep, splev
#from geometric_controller.msg import target_positions


class squircleworld_NF(object):
    def __init__(self, name_of_uav, name_space, velocity, initial_hovering_height, freq): 
        """i1nitializes various paramters
        For 3D rotation is not a full rotation matrix but the script assumes that there will not be 
        a rotation in roll and pitch (assuing yaw is about z), so you can only specify the rotation in yaw  
        at the moment. if otherwise, define a full rotation matrix for each object and pass it on to the 
        function "similarity_transformation.
        self.O: Info of obstacles coordinates of the center, radius, s, orientation, scaling in 3 directions
        self.n: number of grid points in each x, y and z direction 
        self.frame = z-plane (or any direction by tweaking the code) at which the navigation function is plotted 
        self.goal: destination point
        self.k: navigation function keppa 
        self.lambda_sq: parameter for calculating analytical switch of squircle 
        self.lambda_sp: parameter for calculating analytical switch of transformed sphere
        self.rho: radii of transformed spheres
        self.q: self.p = center of squircles and spheres
        self.height: height of the frame at which the navigation function is plotted, used just for plottiing 
        """
        self.hover_at = initial_hovering_height
        self.uav = name_of_uav
        self.namespace  = name_space
        if self.namespace == 'firefly1': 
            thisquad = self.namespace
            otherquad1 = 'firefly2' 
            otherquad2 = 'firefly3'
        elif self.namespace == 'firefly2': 
            thisquad = self.namespace
            otherquad1 = 'firefly1'
            otherquad2 = 'firefly3'
        elif self.namespace == 'firefly3': 
            thisquad = self.namespace
            otherquad1 = 'firefly1'
            otherquad2 = 'firefly2'
        
        
        self.no_of_obstacles = 9 # this number includes workspace boundary
        self.O= [\
                 [0.0, 0.0, 15.0, 15.0, 0.9999, 0.0, 1.0, 1.0, 1.0], \
                 [-6.0, -8.0, 4.05, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.2], \
                 [6.0, -8.0, 1.05, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.8], \
                 [0.0, -4.0, 2.05, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.4], \
                 [-6.0, 0.0, 1.55, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.533], \
                 [6.0, 0.0, 2.55, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.32], \
                 [0.0, 4.0, 1.55, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.533], \
                 [-6.0, 8.0, 3.05, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.266], \
                 [6.0, 8.0, 1.05, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.8] \
                 ]
        self.k = 40.0#13
        self.lambda_sq = 1e8#1e6# 1e4
        self.lambda_sp = 1e8 # 1e4
        """
        self.no_of_obstacles = 5 # this number includes workspace boundary
        self.O= [\
                 [0.0, 0.0, 15.0, 15.0, 0.9999, 0.0, 1.0, 1.0, 1.0], \
                 [0.0, -4.0, 2.05, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.4], \
                 [-6.0, 0.0, 1.55, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.533], \
                 [6.0, 0.0, 2.55, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.32], \
                 [0.0, 4.0, 1.55, 0.8, 0.9999, 0.0, 1.0, 1.0, 0.533] 
                ]
        self.k = 14.0#13
        self.lambda_sq = 20#1e6# 1e4
        self.lambda_sp = 1e6 # 1e4
        """         
        self.scale_factor = self.O[0][3] # radius of workspace is the scale factor to be reduced to 1
        for i in range(len(self.O)): 
            for j in range(len(self.O[i])): 
                if j <= 3: 
                    self.O[i][j] = self.O[i][j]/self.scale_factor
        # navigation fnction parameters
        self.n = 5
        self.goal = np.array([0.0, 0.0, 0.1])
        open("navigation_function_for_{}.txt".format(self.namespace), "w").close()
        open("goal_for_{}.txt".format(self.namespace), "w").close()

        # star to sphere transformation radii and centers
        self.rho = []; self.p = []; self.q = []
        for i in range(len(self.O)): 
            if i == 0: 
                #self.rho.append(self.O[i][3] - 0.005)
                self.rho.append(np.sqrt(3) * self.O[0][3] + 0.5)
                #self.rho.append(np.sqrt(3)*self.O[0][3]/ min(self.O[0][6], self.O[0][7], self.O[0][8]) + 0.005 )
                self.p.append(np.array([self.O[i][0], self.O[i][1], self.O[i][2]]))
                self.q.append(np.array([self.O[i][0], self.O[i][1], self.O[i][2]]))
            else: 
                #self.rho.append(0.05)
                self.rho.append(self.O[i][3] / max(self.O[i][6], self.O[i][7], self.O[i][8] - 0.04))
                self.p.append(np.array([self.O[i][0], self.O[i][1], self.O[i][2]+0.005]))
                self.q.append(np.array([self.O[i][0], self.O[i][1], self.O[i][2]+0.005]))  
                
        #ROS specific
                
        # defining required parameters and lists for final velocity and acceleration in x, y and z direction  
        self.time = time.time()
        self.d0 = 0.1 # a factor to avoid interquadrotor collision
        self.dX = 0.05/self.scale_factor
        self.VelocityMagnitude = velocity # magnitude of desired velocity in m/s
        self.smooth_vx = []; self.smooth_vy = []; self.smooth_vz = [] # wont actully be used, just for getting acc
        self.smooth_ax = []; self.smooth_ay = []; self.smooth_az = []
        self.counter = 0; self.time_points = []#; self.position  = []  
        self.proximity = 0.005; self.planning_counter = 0
        self.phase_one = False # controls switcching of the controller when close to the target
        self.dt = 1.0/freq#; self.position = np.array([0.0,0.0,0.0])
        self.switching_position = 0.005
        self.hd_previous = np.array([0,0,0])
        self.pub = rospy.Publisher('/'+self.namespace+'/NavigationFunctionTrajectory', velocities_from_navigation_function, queue_size = 1, tcp_nodelay = True)
        try:
            rospy.Subscriber('/target_positions_for_quads', target_positions, self.callback1, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/'+thisquad+'/odometry_sensor1/odometry', Odometry, self.callback2, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/'+otherquad1+'/odometry_sensor1/odometry', Odometry, self.callback3, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/'+otherquad2+'/odometry_sensor1/odometry', Odometry, self.callback4, queue_size = 1, tcp_nodelay = True)
            
        except:
            print('problem subscribing to one of the topic above')

    def callback3(self, data): 
        self.x1 = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.x1 = self.x1/self.scale_factor         

    def callback4(self, data): 
        self.x2 = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.x2 = self.x2/self.scale_factor
        
    def callback1(self, tp): 
        """the callback that takes target positions"""
        
        if self.namespace == 'firefly1': 
            self.goal = np.array([tp.robot1.x, tp.robot1.y, tp.robot1.z])/self.scale_factor
        elif self.namespace == 'firefly2': 
            self.goal = np.array([tp.robot2.x, tp.robot2.y, tp.robot2.z])/self.scale_factor
        elif self.namespace == 'firefly3':
            self.goal = np.array([tp.robot3.x, tp.robot3.y, tp.robot3.z])/self.scale_factor
        f1 = open('goal_for_{}.txt'.format(self.namespace), 'a')
        f1.write("%s, %s, %s, %s\n" % (time.time(), self.goal[0]*self.scale_factor, self.goal[1]*self.scale_factor, self.goal[2]*self.scale_factor))   
    def distance_to_goal(self, q): 
        """returns goal function"""
        return nrm(q - self.goal)**2.2#2.2
        
    def calculate_beta (self, a, ss, rr, identifier): 
        """takes all points of the workspace, parameter s and radious r of squircle for 
        every point and returns the value of beta, eventually all this wont be required.
        a: points 
        ss: value of paramter s
        rr: radius of the sqauircle
        identifier: 0 if object is an obstacle and 1 if its workspace
        """
        x = a[:,0]; y = a[:,1]; z = a[:,2]
        temp1 = (2 - 4 * ss**2) * (x * y)**2
        temp2 = (x**2 + y**2 + np.sqrt(x**4 + y**4 + temp1)) / 2.0
        beta_temp = (temp2 + z**2)/2 + sq(temp2**2 + z**4 + (2-4*ss**2)*temp2*z**2)/2 - rr**2
        if identifier == 1: 
            beta_temp = [i if i < 0 else 0 for i in beta_temp]
            beta = [-1.0*j for j in beta_temp]
        else: 
            beta_temp = [i if i > 0 else 0 for i in beta_temp]
            beta = [j for j in beta_temp]

        return np.asarray(beta)


        
       
    def similarity_transformation(self, O, c_): 
        """take the parameters of obstacles or workspace and returns the 
        transformed coordinates in order to calculate the function beta and 
        other necessary things.
        O: the object, it is either workspace or obstacles as defined in _init_ function  
        c: initial grid, g = transformed grid, r: radius, s: parameter
        r and s are transfered just not to change too much right now, not 
        really needed to do it here
        """

        r = O[3]; s = O[4]
        scale = np.array([[O[6], 0, 0],[0, O[7], 0], [0, 0, O[8]]])
        rotation = np.array(([cos(O[5]), sin(O[5]), 0], [-sin(O[5]), cos(O[5]), 0], [0, 0, 1]))
        translation = np.array([O[0], O[1], O[2]])
        # doesnt rotate at the moment
        d = c_ - translation
        e = np.dot(d, scale) 
        return e, s, r

    def calculate_analytic_switch(self, _c, b): 
        """calculate the analytic switch"""
        p = [self.distance_to_goal(i) for i in _c]
        beta_bar0 = [[reduce(lambda x, y: x*y, b[j][:i] + b[j][i+1:]) for i in range(len(b[j]))]for j in range(len(b))]
        _s = [[p[j] * beta_bar0[j][k] / (p[j] * beta_bar0[j][k] + self.lambda_sq * b[j][k]) for k in range(len(beta_bar0[j]))] for j in range(len(b))]

        return _s

    def translated_scaling_map(self, _c, b): 
        """calculate star to sphere transformation
        """
        max_betas = [max(b[i]) for i in range(len(b))]
        #v = [[np.sqrt(1 + b[i][j]/max_betas[i]) * self.rho[j] / nrm(_c[i] - self.q[j]) for j in range(len(b[i]))] for i in range(len(b))]
        v = [[np.sqrt(1 + b[i][j]) * self.rho[j] / nrm(_c[i] - self.q[j]) for j in range(len(b[i]))] for i in range(len(b))]
        T = [[v[i][j] * (_c[i]- self.q[j]) + self.p[j] for j in range(len(b[i]))] for i in range(len(b))]
        
        return T
        
    def star_to_sphere_transform(self, _c, sd, TT):
        """calculate star to sphere transformation by linear combination of translated scaling"""
        TTd = [i for i in _c]
        ssd = [1-reduce(add, sd[i]) for i in range(len(sd))]
        total = [reduce(add, map(lambda x,y:x*y, sd[i], TT[i])) for i in range(len(sd))]
        hh = [ssd[i]*TTd[i] + total[i] for i in range(len(sd))]
        return hh
        
        
    def beta_for_sphereworld(self, _q): # function to return beta values
        b = [self.rho[i]**2 - nrm(_q - self.p[i])**2 if i==0 else -self.rho[i]**2 + nrm(_q - self.p[i])**2 for i in range(self.no_of_obstacles)]
        b = [0 if i < 0 else i for i in b]        
        beta = reduce(mul, b)
        return beta
    
    def navigation_function(self, hh): 
        """calculate navigation function"""
        Beta = [self.beta_for_sphereworld(hh[i]) for i in range(len(hh))]
        a = [self.distance_to_goal(hh[i]) for i in range(len(hh))]
        b = [pwr(a[i], self.k) + self.lambda_sp * Beta[i] for i in range(len(hh))]
        phi = [a[i] / pwr(b[i], 1.0 / self.k) for i in range(len(hh))]
        phi = [i if i < 1 else 1.0 for i in phi]        
        return phi



    def generate_position_trajectory(self, X_, odomm): 
        
        msg = velocities_from_navigation_function()
        msg.header.stamp = odomm.header.stamp#rospy.Time.now()
        
        direction_vector  = (self.goal-X_)/nrm(self.goal-X_)
        #direction_vector = np.array([1,0,0])
        position = self.scale_factor*X_
    
        msg.ddes.x = 1#direction_vector[0]
        msg.ddes.y = 0#direction_vector[1]
        msg.ddes.z = 0#direction_vector[2]
 
        #msg.ddes_dot.x = 0#direction_vector[0]
        #msg.ddes_dot.y = 0#direction_vector[1]
        #msg.ddes_dot.z = 0#direction_vector[2]       
            
        msg.pdes.x = self.initial_position[0]
        msg.pdes.y = self.initial_position[1]
        if self.initial_position[2] < 3: 
            msg.pdes.z = self.hover_at#initial_position[3]  
        else: 
            msg.pdes.z = self.initial_position[2]

        msg.vdes.x = 0; msg.vdes.y = 0; msg.vdes.z = 0
        
        msg.ades.x = 0; msg.ades.y = 0; msg.ades.z = 0 
        #msg.jdes.x = 0; msg.jdes.y = 0; msg.jdes.z = 0

        msg.controller = 0 # 1: velocity controller, 0: postion controller
        #f0 = open('desired_trajectory.txt', 'a')
        #f0.write("%s, %s, %s,  %s, %s, %s, %s, %s, %s, %s,  %s, %s, %s, %s, %s, %s\n" % (msg.pdes.x, msg.pdes.y, msg.pdes.z, msg.vdes.x, msg.vdes.y, msg.vdes.z, 0.0, msg.ades.x, msg.ades.y, msg.ades.z, msg.jdes.x, msg.jdes.y, msg.jdes.z, msg.sdes.x, msg.sdes.y, msg.sdes.z))
        self.pub.publish(msg) 
        self.position = np.array([position[0], position[1], position[2]])
        self.time = time.time()


    def generate_velocity_trajectory(self, phi_, X_, odom): 

        self.counter = self.counter+1
        
        current_time = time.time()
        t = current_time - self.time
        self.time_points.append(t)
        #delt = 0.01#(max(self.time_points)-min(self.time_points))/self.counter
        
        Vtarget = np.array([0, 0, 0]) # velocity of the target in m/sec
        dist_to_goal = nrm(X_-self.goal)

        msg = velocities_from_navigation_function()
        msg.header.stamp = odom.header.stamp#rospy.Time.now()
        self.heading_direction = np.array([1,0,0])

        msg.ddes.x = self.heading_direction[0]
        msg.ddes.y = self.heading_direction[1]
        msg.ddes.z = self.heading_direction[2]
        

        cx = (self.n+1)/2
        cy = (3*self.n+1)/2
        cz = (5*self.n+1)/2 
        time_factor = 1/(1+2*np.exp(-2*(t-1)))
        distance_to_goal_factor = scipy.special.erf(6.0*(dist_to_goal-self.proximity)) 
        #d1 = np.linalg.norm(self.x1-X_); d2 = np.linalg.norm(self.x2-X_)
        #rospy.loginfo('d1 and d2 are:%f, %f',d1, d2)
        #collision_factor1 = 1.0/(1.0+np.exp(-100*(d1-self.d0))); collision_factor2 = 1.0/(1.0+np.exp(-100*(d2-self.d0)))
        smoothing_factor = time_factor * distance_to_goal_factor #* collision_factor1 * collision_factor2 
        
        if nrm(X_-self.goal) > self.proximity and self.phase_one == False:#0#direction_vector[2]#1#direction_vector[0

            dNdX = np.gradient(phi_)
            dNdX = [-i for i in dNdX]
            v1 = np.array([dNdX[cx], dNdX[cy], dNdX[cz]])
            if nrm(v1) == 0: 
                v = 0.8*self.v_previous
                rospy.loginfo("the invalid goal for %s is: %f, %f, %f", self.namespace, self.goal[0], self.goal[1], self.goal[2])
            else: 
                v = smoothing_factor * (self.VelocityMagnitude * v1/nrm(v1))
        
            self.smooth_vx.append(v[0]); self.smooth_vy.append(v[1]); self.smooth_vz.append(v[2])
          
            self.position = self.position + v*self.dt

            if self.counter == 1:
                msg.pdes.x = self.position[0]; msg.pdes.y = self.position[1]; msg.pdes.z = self.position[2]                
                msg.vdes.x = v[0]; msg.vdes.y = v[1]; msg.vdes.z = v[2]                 
                msg.ades.x = 0; msg.ades.y = 0; msg.ades.z = 0 
               
                msg.controller = 0 # 1: velocity controller, 0: position controller
                self.v_previous = np.array([v[0], v[1], v[2]])

        
            elif self.counter < 10: 
                msg.pdes.x = self.position[0] 
                msg.pdes.y = self.position[1] 
                msg.pdes.z = self.position[2] 

                msg.vdes.x = v[0]; msg.vdes.y = v[1]; msg.vdes.z = v[2] 
      
                msg.ades.x = (v[0]-self.v_previous[0])/self.dt
                msg.ades.y = (v[1]-self.v_previous[1])/self.dt
                msg.ades.z = (v[2]-self.v_previous[2])/self.dt

                #msg.jdes.x = (msg.ades.x-self.a_previous[0])/self.dt
                #msg.jdes.y = (msg.ades.y-self.a_previous[1])/self.dt
                #msg.jdes.z = (msg.ades.z-self.a_previous[2])/self.dt  
                
                msg.controller = 0 # 1: velocity controller, 0: postion controller
                #self.a = self.time_points[-1]
                self.v_previous = np.array([v[0], v[1], v[2]])
                #self.a_previous = np.array([msg.ades.x, msg.ades.y, msg.ades.z])

            else: 
                
                msg.pdes.x = self.position[0] 
                msg.pdes.y = self.position[1] 
                msg.pdes.z = self.position[2] 

                msg.vdes.x = v[0]; msg.vdes.y = v[1]; msg.vdes.z = v[2] 
      
                msg.ades.x = (v[0]-self.v_previous[0])/self.dt
                msg.ades.y = (v[1]-self.v_previous[1])/self.dt
                msg.ades.z = (v[2]-self.v_previous[2])/self.dt
                
                #msg.jdes.x = (msg.ades.x-self.a_previous[0])/self.dt
                #msg.jdes.y = (msg.ades.y-self.a_previous[1])/self.dt
                #msg.jdes.z = (msg.ades.z-self.a_previous[2])/self.dt 
            
                msg.controller = 0 # 1: velocity controller, 0: postion controller
                #self.a = self.time_points[-1]
                self.v_previous = np.array([v[0], v[1], v[2]])                
                #self.a_previous = np.array([msg.ades.x, msg.ades.y, msg.ades.z])
                """
                t = list(np.linspace(min(self.time_points), max(self.time_points), self.counter))
      
                if self.counter > 20: # number of points used in making spline 
                    t.pop(0); self.smooth_vx.pop(0); self.smooth_vy.pop(0); self.smooth_vz.pop(0); self.time_points.pop(0)
                    t = list(np.linspace(min(self.time_points), max(self.time_points), 20))
                    
                a = splrep(t,self.smooth_vx,k=5,s=4); b = splrep(t,self.smooth_vy,k=5,s=4); c = splrep(t,self.smooth_vz,k=5,s=4)
                a2 = splev(t,a,der=1); b2 = splev(t,b,der=1); c2 = splev(t,c,der=1)
                
                msg.pdes.x = self.position[0] 
                msg.pdes.y = self.position[1] 
                msg.pdes.z = self.position[2] 

                msg.vdes.x = v[0]; msg.vdes.y = v[1]; msg.vdes.z = v[2] 
                
                msg.ades.x = a2[-1]; msg.ades.y = b2[-1]; msg.ades.z = c2[-1]


                msg.controller = 0 # 1: velocity controller, 0: postion controller
                self.switching_position = X_*self.scale_factor
                self.v_previous =np.array([v[0], v[1], v[2]])
                """
                
        else: 
            #self.phase_one = True
            position = self.switching_position

            msg.pdes.x = self.position[0]; msg.pdes.y = self.position[1]; msg.pdes.z = self.position[2]
            
            msg.vdes.x = Vtarget[0]; msg.vdes.y = Vtarget[1]; msg.vdes.z = Vtarget[2]
            
            msg.ades.x = 0; msg.ades.y = 0; msg.ades.z = 0
           
            msg.controller = 0 # 1: velocity controller, 0: postion controller
            print 'the quad has switched'


        self.pub.publish(msg)
            
    def callback2(self, odom): 
        """construct navigation function"""
        Xprime = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        #self.current_position = Xprime
        X = Xprime/self.scale_factor 

        if self.planning_counter  == 0: 
            self.initial_position = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        elif self.planning_counter > 0 and self.planning_counter < 100: 
            self.generate_position_trajectory(X, odom)
        else: 
            #discretization in x direction 
            x1 = np.linspace(X[0]-self.dX, X[0]+self.dX, self.n); y1 = np.array([X[1]]); z1 = np.array([X[2]])
            c1 = np.vstack(np.meshgrid(x1, y1, z1)).reshape(3, -1).T
    
            #discretization in y direction 
            x2 = np.array([X[0]]); y2 = np.linspace(X[1]-self.dX, X[1]+self.dX, self.n);  z2 = np.array([X[2]])
            c2 = np.vstack(np.meshgrid(x2, y2, z2)).reshape(3, -1).T
            
            #discretization in z direction 
            x3 = np.array([X[0]]); y3 = np.array([X[1]]); z3 = np.linspace(X[2]-self.dX, X[2]+self.dX, self.n)
            c3 = np.vstack(np.meshgrid(x3, y3, z3)).reshape(3, -1).T
    
            c = np.concatenate((c1,c2,c3), axis = 0)
        
            # calculate beta funtion for each obstacle
            beta = []
            for i in range(self.no_of_obstacles): 
                identifier = 1 if i==0 else 0
                g, s, r = self.similarity_transformation(self.O[i], c)
                beta.append(self.calculate_beta(g, s, r, identifier))
    
            # function construction
            beta = zip(*beta)
            s = self.calculate_analytic_switch(c, beta)   
            T = self.translated_scaling_map(c, beta)
            h = self.star_to_sphere_transform(c, s, T)
            #phi = self.navigation_function(h)

            """
            if self.goal[1] <= 0.6 and self.goal[1] >= -0.6: 
                rospy.loginfo("the target has not turned")
                self.goal = self.goal + np.array([0, -0.5/self.scale_factor, 0])*self.dt
            elif self.goal[1] <= -0.6: 
                rospy.loginfo("the target has turned")
                self.goal = self.goal + np.array([-0.5/self.scale_factor, 0, 0])*self.dt
            """

            #rospy.loginfo("heading direction is %f, %f, %f", self.heading_direction[0], self.heading_direction[1], self.heading_direction[2])
            phi = self.navigation_function(h)
            f = open('navigation_function_for_{}.txt'.format(self.namespace), 'a')
            f.write("%s, %s\n" % (time.time(), phi[2]))
            self.generate_velocity_trajectory(phi, X, odom)
        
        self.planning_counter += 1

if __name__ == '__main__':
    name = rospy.get_param('uav_name')
    name_space = rospy.get_param('namespace')
    velocity = rospy.get_param('uav_velocity')
    freq = rospy.get_param('publish_frequency')
    initial_hovering_height = rospy.get_param('initial_hovering_height')
    rospy.init_node('navfunc3d_nobstacle', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(freq)
    start_time = time.time()
    traj = squircleworld_NF(name, name_space, velocity, initial_hovering_height, freq)
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass


