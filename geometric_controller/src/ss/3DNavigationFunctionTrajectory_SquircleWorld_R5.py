#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 14:50:16 2017
This script generates navigation function in 3D in a sphere world
@author: indrajeet
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import rospy
import numpy as np 
import time 
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import cm
from numpy.linalg import norm as nrm
from numpy import sqrt as sq
from numpy import power as pwr
from numpy import pi as pi
from numpy import sin as sin
from numpy import cos as cos 
from scipy.interpolate import splrep, splev
from pyquaternion import Quaternion 
import scipy
import message_filters
from gazebo_msgs.msg import LinkStates
from isy_geometric_controller.msg import velocities_from_navigation_function
#from isy_geometric_controller.msg import Desired_Trajectory
from nav_msgs.msg import Odometry

class NavigationFunction_Trajectory3D(object):
    def __init__(self, name_of_uav, time_instance): 
        """i1nitializes various paramters
        For 3D rotation is not a full rotation matrix but the script assumes that there will not be 
        a rotation in roll and pitch (assuing yaw is about z), so you can only specify the rotation in yaw  
        at the moment. if otherwise, define a full rotation matrix for each object and pass it on to the 
        function "similarity_transformation.
        self.On: Parameters for obstacles coordinates of the center, radius, s, orientation, scaling in 3 directions
        self.WS: parameters of workspace boundary similarly defined
        self.n: number of grid points in each x, y and z direction 
        self.frame = z-plane (or any direction by tweaking the code) at which the navigation function is plotted 
        self.goal: destination point
        self.k: navigation function keppa 
        self.lambda_sq: parameter for calculating analytical switch of squircle 
        self.lambda_sp: parameter for calculating analytical switch of transformed sphere
        self.rho: radii of transformed spheres
        self.q: self.p = center of squircles and spheres
        self.height: height of the frame at which the navigation function is plotted, used just for plottiing 
        self.dX: intervl within which the navigation function should be calculated for numerically determining 
        velocities, acceleration and jerk
        controller phases: 
        phase_one: Lee's velocity controller with velocities from navigation function 
        phase_two: Lee's positon controller with velocities from navigation function keeps reducing to zero 
        phase_three: Lee's position controller with zero velocities 
        NOTE: CHANGE THE ZERO VELOCITY TO THE ESTIMATED VELOCITY OF THE TARGET 
        """
        self.uav = name_of_uav
        """
        # define the squircle obstacle by: x and y of center, radius, s, orientation, scaling in x and y
        self.WS = np.array([0.0, 0.0, 1, 1.0, 0.995, 0, 1.0, 1.0, 1.0]) # workspace
        self.O1 = np.array([0.5, 0.5, 0.2, 0.2, 0.995, 0, 1.0, 1.0, 1.0]) # obstacle 1
        self.O2 = np.array([-0.5, -0.5, 0.2, 0.2, 0.995, 0, 1.0, 1.0, 1.0]) # obstacle 2
        self.O3 = np.array([0.5, -0.5, 0.25, 0.25, 0.995, 0, 1.0, 1.0, 1.0]) # obstacle 3
        self.O4 = np.array([0, 0, 0.075, 0.075, 0.995, 0, 1.0, 1.0, 1.0]) # obstacle 4
        
        #self.O1 = np.array([0.4, 0.4, 0.2, 0.95, pi/3, 1.0, 1.0]) # obstacle 1
        #self.O2 = np.array([-0.5, -0.5, 0.2, 0.95, pi/9, 1.0, 1.0]) # obstacle 2
        #self.WS = np.array([0.0, 0.0, 1.0, 0.95, 0, 1.0, 1.0]) # workspace    
        
        self.time = time.time(); self.scale_factor = 10.0 # scaling between actual WS and navigation function WS
        self.n = 5; self.frame = 1; self.dX = 0.0005/self.scale_factor
        self.goal = np.array([0., -0.6, 0.2]); self.k = 10.0; self.lambda_sq = 10; self.lambda_sp = 10
        #self.k = 4; self.lambda_sq = 1; self.lambda_sp = 1
        self.height = (self.WS[2]-self.WS[3]) + (2*self.WS[3]/(self.n-1))*self.frame
        
        # star to sphere transformation radii
        self.rho0 = np.sqrt(3) * self.WS[3] + 0.5 
        self.rho1 = self.O1[3] - 0.05
        self.rho2 = self.O2[3] - 0.05
        self.rho3 = self.O3[3] - 0.05
        self.rho4 = self.O4[3] - 0.05       
        #center of star and spheres used in transformations
        self.p0 = self.q0 = np.array([self.WS[0], self.WS[1], self.WS[2]]) 
        self.p1 = self.q1 = np.array([self.O1[0], self.O1[1], self.O1[2]]) 
        self.p2 = self.q2 = np.array([self.O2[0], self.O2[1], self.O2[2]]) 
        self.p3 = self.q3 = np.array([self.O3[0], self.O3[1], self.O3[2]]) 
        self.p4 = self.q4 = np.array([self.O4[0], self.O4[1], self.O4[2]])
        """
        # define the squircle obstacle by: x and y of center, radius, s, orientation, inverse of scaling in x, y and z
        #self.WS = np.array([0.0, 0.0, 1, 1.0, 0.995, 0, 1.0, 1.0, 1.0]) # workspace
        self.WS = np.array([0.0, 0.0, 0.25, 0.25, 0.995, 0, 0.25, 0.25, 1.0]) # workspace
        self.O1 = np.array([0.5, 0.5, 0.2, 0.2, 0.995, 0, 1.0, 1.0, 1.0]) # obstacle 1
        self.O1 = np.array([-0.5, -0.5, 0.1005, 0.1, 0.995, 0, 0.5, 0.5, 1.0]) # house1
        self.O2 = np.array([0.5, -0.5, 0.1005, 0.1, 0.995, 0, 0.5, 0.5, 1.0]) # house1
        self.O3 = np.array([0, 0.5, 0.2005, 0.075, 0.995, 0, 1.0, 1.0, 0.375]) # radio tower
    
        #self.O1 = np.array([0.4, 0.4, 0.2, 0.95, pi/3, 1.0, 1.0]) # obstacle 1
        #self.O2 = np.array([-0.5, -0.5, 0.2, 0.95, pi/9, 1.0, 1.0]) # obstacle 2
        #self.WS = np.array([0.0, 0.0, 1.0, 0.95, 0, 1.0, 1.0]) # workspace    
        
        
        self.n = 5; self.frame = 1
        #self.goal = np.array([0.75, 0.75, 0.1]); self.k = 4.5; self.lambda_sq = 1.0; self.lambda_sp = 30.0
        self.goal = np.array([0.75, 0.75, 0.1]); self.k = 6; self.lambda_sq = 1e4; self.lambda_sp = 1e4
        self.height = (self.WS[2]-self.WS[3]) + (2*self.WS[3]*self.frame)/(self.n-1)
        
        # star to sphere transformation radii
        self.rho0 = np.sqrt(3)*self.WS[3]/ min(self.WS[6], self.WS[7], self.WS[8]) + 0.25
        self.rho1 = self.O1[3] / max(self.O1[6], self.O1[7], self.O1[8]) - 0.09
        self.rho2 = self.O2[3] / max(self.O2[6], self.O2[7], self.O2[8]) - 0.09
        self.rho3 = self.O3[3] / max(self.O3[6], self.O3[7], self.O3[8]) - 0.07
        #self.rho4 = self.O4[3] - 0.05        
        #center of star and spheres used in transformations
        self.p0 = self.q0 = np.array([self.WS[0], self.WS[1], self.WS[2]]) 
        self.p1 = self.q1 = np.array([self.O1[0], self.O1[1], self.O1[2]]) 
        self.p2 = self.q2 = np.array([self.O2[0], self.O2[1], self.O2[2]]) 
        self.p3 = self.q3 = np.array([self.O3[0], self.O3[1], self.O3[2]]) 
        #self.p4 = self.q4 = np.array([self.O4[0], self.O4[1], self.O4[2]]) 
        
        # defining required parameters and lists for final velocity and acceleration in x, y and z direction  
        self.time = time.time(); self.scale_factor = 10.0 # scaling between actual WS and navigation function WS
        self.dX = 0.001#0.08/self.scale_factor
        self.VelocityMagnitude = 2 # magnitude of desired velocity in m/s
        self.smooth_vx = []; self.smooth_vy = []; self.smooth_vz = [] # wont actully be used, just for getting acc
        self.smooth_ax = []; self.smooth_ay = []; self.smooth_az = []
        self.counter = 0; self.time_points = []; self.position  = []  
        self.proximity = 0.01; self.dummy_counter = 1.0
        self.phase_one = False; self.phase_two = False; self.phase_three = False # 3 phases of controller 
        
        self.pub = rospy.Publisher('/NavigationFunctionTrajectory', velocities_from_navigation_function, queue_size = 100, tcp_nodelay = False)
        try:
            odom = message_filters.Subscriber('/'+self.uav+'/vi_sensor/ground_truth/odometry', Odometry, tcp_nodelay = False)
            turtle = message_filters.Subscriber('/turtlebot/ground_truth/state', Odometry, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([odom, turtle], 100)
            ts.registerCallback(self.callback)
            rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
 
    def distance_to_goal(self, q): 
        """takes current position and returns the distance to goal function"""
        """TODO: Decide which function should be goal function"""        
        dist_to_goal = nrm(q - self.goal)**2 
        #dist_to_goal = nrm(q - self.goal)**4
        #dist_to_goal = nrm(q - self.goal)**(2 * self.k) 
        return dist_to_goal
        
    def calculate_beta_squircle (self, a, ss, rr, identifier): 
        """takes all points of the workspace, parameter s and radious r of squircle for 
        every point and returns the value of beta, eventually all this wont be required.
        a: points 
        ss: value of paramter s
        rr: radius of the sqauircle
        identifier: 0 if object is an obstacle and 1 if its workspace
        """
        beta = []
        for i in range(len(a)): 
            x = a[:,0][i]; y = a[:,1][i]; z = a[:,2][i]
            temp1 = (2 - 4 * ss**2) * (x * y)**2
            temp2 = (x**2 + y**2 + np.sqrt(x**4 + y**4 + temp1)) / 2
            beta_temp = (temp2 + z**2)/2 + sq(temp2**2 + z**4 + (2-4*ss**2)*temp2*z**2)/2 - rr**2
            if identifier == 1 and beta_temp >= 0:
                beta_temp = 0
            elif identifier == 0 and beta_temp <= 0: 
                beta_temp = 0
            
            if identifier == 1: 
                beta.append(-beta_temp)
            else: 
                beta.append(beta_temp)

        #if identifier == 1: 
        #    beta = [-i for i in beta]
           
        return beta      
      

        
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
        #scale_inv = np.linalg.inv(scale)        
        # transformation 

        # doesnt rotate at the moment
        d = c_ - translation
        e = np.dot(d, scale) 
        return e, r, s
    
    def calculate_analytic_switch(self, c_, beta0, beta1, beta2, beta3): 
        """calculate the analytic switch"""
        
        beta_bar_0 = []; beta_bar_1 = []; beta_bar_2 = []; beta_bar_3 = []#; beta_bar_4 = []
        _s0 = []; _s1 = []; _s2 = []; _s3 = []; _sd = []; pp  = []
        #a = 2e9
        #a = 120000#self.k#7e3
        for i in range(len(c_)): 
    
            beta_bar_0.append(beta1[i] * beta2[i] * beta3[i]) 
            beta_bar_1.append(beta0[i] * beta2[i] * beta3[i])
            beta_bar_2.append(beta0[i] * beta1[i] * beta3[i])
            beta_bar_3.append(beta0[i] * beta1[i] * beta2[i])
            #beta_bar_4.append(beta0[i] * beta1[i] * beta2[i] * beta3[i])

            current_position = c_[i]
            p = self.distance_to_goal(current_position) 
            pp.append(p)
            _s0.append(p * beta_bar_0[-1] / (p * beta_bar_0[-1] + self.lambda_sq * beta0[i]))
            _s1.append(p * beta_bar_1[-1] / (p * beta_bar_1[-1] + self.lambda_sq * beta1[i]))
            _s2.append(p * beta_bar_2[-1] / (p * beta_bar_2[-1] + self.lambda_sq * beta2[i]))
            _s3.append(p * beta_bar_3[-1] / (p * beta_bar_3[-1] + self.lambda_sq * beta3[i]))
            #_s4.append(p * beta_bar_4[-1] / (p * beta_bar_4[-1] + self.lambda_sq * beta4[i]))
            _sd.append(1 - (_s0[-1] + _s1[-1] + _s2[-1]+ _s3[-1]))
            
        return _s0, _s1, _s2, _s3, _sd
        
    def tranlated_scaling_map(self,c_, beta0, beta1, beta2, beta3): 
        """calculate star to sphere transformation
        formulae as per koditcheck didnot seem to work, caili's formulation is used instead to try
        """
        TT0 = []; TT1 = []; TT2 = []; TT3 = []#; TT4 = []
        #a = max(beta0)
        for i in range(len(c_)):
            current_position = c_[i]
            #v0 = np.sqrt(1 + beta0[i]) * self.rho0 / nrm(current_position - self.q0)
            #v1 = np.sqrt(1 + beta1[i]) * self.rho1 / nrm(current_position - self.q1)
            #v2 = np.sqrt(1 + beta2[i]) * self.rho2 / nrm(current_position - self.q2)
            v0 = np.sqrt(1 + beta0[i]) * self.rho0 / nrm(current_position - self.q0)
            v1 = np.sqrt(1 + beta1[i]) * self.rho1 / nrm(current_position - self.q1)
            v2 = np.sqrt(1 + beta2[i]) * self.rho2 / nrm(current_position - self.q2)
            v3 = np.sqrt(1 + beta3[i]) * self.rho3 / nrm(current_position - self.q3)
            #v4 = np.sqrt(1 + beta4[i]) * self.rho4 / nrm(current_position - self.q4) 
            
            TT0.append(v0 * (current_position - self.q0) + self.p0)
            TT1.append(v1 * (current_position - self.q1) + self.p1)
            TT2.append(v2 * (current_position - self.q2) + self.p2)
            TT3.append(v3 * (current_position - self.q3) + self.p3)
            #TT4.append(v4 * (current_position - self.q4) + self.p4)
        """ASK: what about translated center of the workspace and obstacles"""
        
        return TT0, TT1, TT2, TT3#, TT4
        
    def star_to_sphere_transform(self, c_, ss0, ss1, ss2, ss3, ssd, TT0, TT1, TT2, TT3):
        """calculate star to sphere transformation by linear combination of translated scaling"""
        hh = [] 
        for i in range(len(c_)):
            TTd = c_[i] # destination translated scaling, identity map as per Koditchek
            #print 'TTd', TTd
            hh.append(ssd[i] * TTd + (ss0[i] * TT0[i] + ss1[i] * TT1[i] + ss2[i] * TT2[i] + ss3[i] * TT3[i]))
        return hh
        
    def beta_for_sphereworld(self, _q): # function to return beta values
        _b0 = self.rho0**2 - nrm(_q - self.p0)**2
        _b1 = -self.rho1**2 + nrm(_q - self.p1)**2
        _b2 = -self.rho2**2 + nrm(_q - self.p2)**2
        _b3 = -self.rho3**2 + nrm(_q - self.p3)**2
        #_b4 = -self.rho4**2 + nrm(_q - self.p4)**2
        beta = _b0 * _b1 * _b2 * _b3 #* _b4
        return beta, _b0, _b1, _b2, _b3#, _b4
    
    def navigation_function(self, hh): 
        """calculate navigation function"""
        phi0 = [];  betaa = []
        #print 'hh', hh
        for i in range(len(hh)):
            #print i, hh[i]
            Beta, b0, b1, b2, b3 = self.beta_for_sphereworld(hh[i])
            betaa.append(Beta)
            
            a = self.distance_to_goal(hh[i])
            #print 'in nav func calcs', b0,b1,b2,b3, a
            b = pwr(a, self.k)+ self.lambda_sp * Beta
            #bb.append(b4)
            phi00 = a / pwr(b, 1.0 / self.k)
            if phi00 >1: 
                phi00 = 1
            phi0.append(phi00)
            #phi0.append(a / pwr(b, 1.0 / self.k))

        return phi0
     
 
    def calculate_velocity_trajectory(self, phi_, X_): 
        """
        This function calculates the velocities for the quadrotor by numerically differentiating 
        the navigation function in the vicinity of the current position. 
        The vicinity is given by variable self.dX and the number of discretization points by self.n 
        in the callback function. 
        For example (all this is in callback), 11 points are taking within 0.1m of the current x, y 
        and z position and a grid is made at which value of navigation function is calculated. 
        This function then takes those values as a list and calculate all required derivative 
        using numpy's gradient function. Subsequently it publishes these values at the center point of the grid. 
        """
        #print 'hi'
        self.counter = self.counter+1
        
        current_time = time.time()
        t = current_time - self.time
        self.time_points.append(t)
        delt = (max(self.time_points)-min(self.time_points))/self.counter
        
        Vtarget = np.array([0, 0, 0]) # velocity of the target in m/sec
        #self.goal = self.goal - Vtarget*delt#np.array([self.goal[0], self.goal[1]-Vtarget*delt, self.goal[2]])
        #print self.goal
        
        dist_to_goal = nrm(X_-self.goal)
        
        msg = velocities_from_navigation_function()
        msg.header.stamp = rospy.Time.now()
               
        direction_vector = (self.goal-X_)/nrm(self.goal-X_)
        # no matter what desired direction wont change        
        msg.desired_direction.x = direction_vector[0]
        msg.desired_direction.y = direction_vector[1]
        msg.desired_direction.z = direction_vector[2]
        

        #t = time.time()-self.time
        
        if nrm(X_-self.goal) > self.proximity and self.phase_one == False:
            
            cx = (self.n+1)/2
            cy = (3*self.n+1)/2
            cz = (5*self.n+1)/2
            
            dNdX = np.gradient(phi_)
            dNdX = [-i for i in dNdX]
            v1 = np.array([dNdX[cx], dNdX[cy], dNdX[cz]])
            #print 'v1', v1
            

            
            #smoothing_factor = scipy.special.erf(0.5*t)*scipy.special.erf(20*(dist_to_goal-self.proximity))
            smoothing_factor = 1/(1+3*np.exp(-2*(t-2)))*scipy.special.erf(20*(dist_to_goal-self.proximity))
            v = smoothing_factor * (self.VelocityMagnitude * v1/nrm(v1))
            self.smooth_vx.append(v[0]); self.smooth_vy.append(v[1]); self.smooth_vz.append(v[2])
            
            f0 = open('desired_velocity.txt', 'a')
            f0.write("%s, %s, %s, %s\n" % (v[0], v[1], v[2], smoothing_factor))
        
 
            if self.counter == 1:
               
                msg.desired_velocity.x = v[0]
                msg.desired_velocity.y = v[1]
                msg.desired_velocity.z = v[2] 
                
                msg.desired_acceleration.x = 0
                msg.desired_acceleration.y = 0
                msg.desired_acceleration.z = 0 
                
                msg.desired_jerk.x = 0
                msg.desired_jerk.y = 0
                msg.desired_jerk.z = 0
                
                msg.controller = 1 # 1: velocity controller, 0: position controller

                
                self.vx_previous = v[0]; self.vy_previous = v[1]; self.vz_previous = v[2]
        
            elif self.counter < 5: 
                #delt = (max(self.time_points)-min(self.time_points))/self.counter            

                msg.desired_velocity.x = v[0]
                msg.desired_velocity.y = v[1]
                msg.desired_velocity.z = v[2] 
                
                
                msg.desired_acceleration.x = 0.1*(v[0]-self.vx_previous)/delt
                msg.desired_acceleration.y = 0.1*(v[1]-self.vy_previous)/delt
                msg.desired_acceleration.z = 0.1*(v[2]-self.vz_previous)/delt 
                
                msg.desired_jerk.x = 0
                msg.desired_jerk.y = 0
                msg.desired_jerk.z = 0
                
                msg.controller = 1 # 1: velocity controller, 0: postion controller
                self.a = self.time_points[-1]

            else: 
                t = list(np.linspace(min(self.time_points), max(self.time_points), self.counter))
                #delt = (max(self.time_points)-min(self.time_points))/self.counter
      
                if self.counter > 100: # number of points used in making spline 
                    t.pop(0); self.smooth_vx.pop(0); self.smooth_vy.pop(0); self.smooth_vz.pop(0)
                    t = list(np.linspace(min(self.time_points), max(self.time_points), 100))
                    
                a = splrep(t,self.smooth_vx,k=4,s=3); b = splrep(t,self.smooth_vy,k=4,s=3); c = splrep(t,self.smooth_vz,k=4,s=3)
                aa = splev(t,a,der=1); bb = splev(t,b,der=1); cc = splev(t,c,der=1)
                aaa = splev(t,a,der=2); bbb = splev(t,b,der=2); ccc = splev(t,c,der=2)

                msg.desired_velocity.x = v[0]
                msg.desired_velocity.y = v[1]
                msg.desired_velocity.z = v[2] 
                
                msg.desired_acceleration.x = aa[-1]
                msg.desired_acceleration.y = bb[-1]
                msg.desired_acceleration.z = cc[-1]
                
                msg.desired_jerk.x = aaa[-1]
                msg.desired_jerk.y = bbb[-1]
                msg.desired_jerk.z = ccc[-1] 
                
                msg.controller = 1 # 1: velocity controller, 0: postion controller
                self.switching_position = self.scale_factor*X_
        else: 
            self.phase_one = True

            msg.desired_velocity.x = Vtarget[0]
            msg.desired_velocity.y = Vtarget[1]
            msg.desired_velocity.z = Vtarget[2]
            
            position = self.switching_position
            #position = self.scale_factor*(self.goal)# - 0.025*direction_vector)
            msg.desired_position.x = position[0]
            msg.desired_position.y = position[1]
            msg.desired_position.z = position[2] 
            
            msg.desired_acceleration.x = 0
            msg.desired_acceleration.y = 0
            msg.desired_acceleration.z = 0
            
            msg.desired_jerk.x = 0
            msg.desired_jerk.y = 0
            msg.desired_jerk.z = 0 
            
            msg.controller = 1 # 1: velocity controller, 0: postion controller
            print 'it switched'

        self.pub.publish(msg)

    def calculate_position_trajectory(self, X_): 
        
        msg = velocities_from_navigation_function()
        msg.header.stamp = rospy.Time.now()
        #direction_vector = (self.goal-X_)/nrm(self.goal-X_)
        position = self.scale_factor*X_
        direction_vector = (self.goal-X_)/nrm(self.goal-X_)
        # no matter what desired direction wont change        
        msg.desired_direction.x = 1#direction_vector[0]
        msg.desired_direction.y = 0#direction_vector[1]
        msg.desired_direction.z = 0#direction_vector[2]
            
        msg.desired_position.x = position[0]
        msg.desired_position.y = position[1]
        msg.desired_position.z = 2           
            
        msg.desired_velocity.x = 0
        msg.desired_velocity.y = 0
        msg.desired_velocity.z = 0
        
        msg.desired_acceleration.x = 0
        msg.desired_acceleration.y = 0
        msg.desired_acceleration.z = 0 
            
        msg.desired_jerk.x = 0
        msg.desired_jerk.y = 0
        msg.desired_jerk.z = 0

        #msg.desired_direction.x = direction_vector[0]
        #msg.desired_direction.y = direction_vector[1]
        #msg.desired_direction.z = direction_vector[2]

        msg.controller = 0 # 1: velocity controller, 0: postion controller
        self.pub.publish(msg) 
        self.time = time.time()
        
       
    def callback(self, odom, turtle): 
        """generate trajectories"""
        
        self.goal = np.array([turtle.pose.pose.position.x, turtle.pose.pose.position.y, turtle.pose.pose.position.z])
        self.goal = self.goal/self.scale_factor + np.array([0,0,0.1])
        Xprime = np.array([odom.pose.pose.position.x-0.1, odom.pose.pose.position.y, odom.pose.pose.position.z+.03])
        X = Xprime/self.scale_factor 
        #print Xprime, X

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

     
        g_ws, r_ws, s_ws = self.similarity_transformation(self.WS, c)
        beta_ws = self.calculate_beta_squircle(g_ws, s_ws, r_ws, 1)  

        g_o1, r_o1, s_o1 = self.similarity_transformation(self.O1, c)
        beta_o1 = self.calculate_beta_squircle(g_o1, s_o1, r_o1, 0)  

        g_o2, r_o2, s_o2 = self.similarity_transformation(self.O2, c)
        beta_o2 = self.calculate_beta_squircle(g_o2, s_o2, r_o2, 0)  

        g_o3, r_o3, s_o3 = self.similarity_transformation(self.O3, c)
        beta_o3 = self.calculate_beta_squircle(g_o3, s_o3, r_o3, 0)  

        #g_o4, r_o4, s_o4 = self.similarity_transformation(self.O4, c)
        #beta_o4 = self.calculate_beta_squircle(g_o4, s_o4, r_o4, 0) 
        
        s0, s1, s2, s3, sd = self.calculate_analytic_switch(c, beta_ws, beta_o1, beta_o2, beta_o3)

        T0, T1, T2, T3 = self.tranlated_scaling_map(c, beta_ws, beta_o1, beta_o2, beta_o3)
        h = self.star_to_sphere_transform(c, s0, s1, s2, s3, sd, T0, T1, T2, T3)
        #print h
        #phi = self.navigation_function(h)
        #print phi
        #self.calculate_velocity_trajectory(phi, X)
            
        if self.dummy_counter <300: 
            self.calculate_position_trajectory(X)
        else: 
            phi = self.navigation_function(h)
            #print h, phi
            self.calculate_velocity_trajectory(phi, X)
        
        self.dummy_counter = self.dummy_counter+1

   

if __name__ == '__main__':
    name = 'firefly'
    rospy.init_node('NavigationFunction_Trajectory', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    start_time = time.time()
    try:
        while not rospy.is_shutdown(): 
            current_time = time.time()
            t = current_time-start_time
            traj = NavigationFunction_Trajectory3D(name, t)
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass


