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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from numpy.linalg import norm as nrm
from numpy import sqrt as sq
from numpy import power as pwr
from plotly.graph_objs import *
from plotly import tools as tls
import plotly.offline as offline
from isy_geometric_controller.msg import velocities_from_navigation_function
from nav_msgs.msg import Odometry
# make shift arrrangement
from geometry_msgs.msg import PoseStamped
class NavigationFunction_Trajectory(object):
    def __init__(self, time_instance): 
        # center and radius of the field, all distances are in meters
        self.q0 = np.array([0,0,5]); self.R0 = 5
        # center and radius of the desired target, all distances are in meters
        #self.qd = np.array([-2,0,4]); self.Rd = 0.25
        self.qd = np.array([-1.5,-1.0,8]); self.Rd = 0.25
        # initial position of the robot
        self.qR = np.array([0,0,0.1])
        # center and radius of 4 obstacles 
        self.N_obstacles = 4
        self.q1 = np.array([1.5, 1.5, 6.5]); self.R1 =  1.5
        self.q2 = np.array([-1.5, 1.5, 5]); self.R2 =  1.5
        self.q3 = np.array([-1.5, -1.5, 3.5]); self.R3 =  1.5
        self.q4 = np.array([1.5, -1.5, 5]); self.R4 =  1.5
        
        self.k = 5 # parameter used in navigation function construction
        self.time = time_instance
        self.VelocityMagnitude = 5.0 # magnitude of desired velocity in m/s
        self.V = []; self.P = []
        self.counter = 0
        #self.pub = rospy.Publisher('/NavigationFunctionVelocities', velocities_from_navigation_function, queue_size = 1)
        self.pub_pelican = rospy.Publisher('/pelican/command/pose', PoseStamped, queue_size = 1)
        try:
            odom = rospy.Subscriber('/pelican/odometry_sensor1/odometry', Odometry, self.generate_velocities)
            rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
        
    def get_beta(self,q): # function to return beta values
        beta0 = self.R0**2 - nrm(q-self.q0)**2
        beta1 = -self.R1**2 + nrm(q-self.q1)**2
        beta2 = -self.R2**2 + nrm(q-self.q2)**2
        beta3 = -self.R3**2 + nrm(q-self.q3)**2
        beta4 = -self.R4**2 + nrm(q-self.q4)**2
        beta = beta0*beta1*beta2*beta3*beta4
        return beta, beta0, beta1, beta2, beta3, beta4
    
    def nav_func(self,q): 
        Beta, b0,b1,b2,b3,b4 = self.get_beta(q)
        dBeta_dq = -2*(q-self.q0)*b1*b2*b3*b4 + \
        2*b0*((q-self.q1)*b2*b3*b4 + (q-self.q2)*b3*b4*b1 + \
        (q-self.q3)*b4*b1*b2 + (q-self.q4)*b1*b2*b3)
        a = nrm(q-self.qd)**2
        b = nrm(q-self.qd)**(2*self.k)+Beta
        c = pwr(b,1.0/self.k)
        phi0 = a/c
        dphi_dq = 2*(q-self.qd)/c - (a/(self.k*b*c))*(2*self.k*pwr(sq(a),2*self.k-2)*(q-self.qd)+dBeta_dq)
        return phi0, dphi_dq

    def generate_velocities(self, odom):
        dt = 0.1
        #msg = velocities_from_navigation_function()
        X = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]);
        phi, grad_phi = self.nav_func(X)
        Vi = -1.0*grad_phi # velocity vector 
        v = Vi/nrm(Vi)
        #print v
        V_bar = self.VelocityMagnitude*Vi/nrm(Vi) # unit vector of velocity 
        #print phi, grad_phi
        if self.counter ==0: 
            Xdesired = X; self.previous_time = time.time()
        else: 
            delt = time.time()-self.previous_time
            #print delt, phi, grad_phi, V_bar*delt
            Xdesired = X + V_bar * dt
            if np.linalg.norm(Xdesired-self.qd)<=0.01:
                print 'hi'
                Xdesired = self.qd
            self.previous_time = time.time()
        #print Xdesired
        #msg.desired_velocity.x = V_bar[0]; msg.desired_velocity.y = V_bar[1]
        #msg.desired_velocity.z = V_bar[2]; self.pub.publish(msg)
        x = Xdesired-X
        f1 = open('vel_nav.txt', 'a')
        f1.write("%s,%s,%s,%s,%s,%s\n" % (X[0],X[1],X[2], Xdesired[0],Xdesired[1],Xdesired[2]))

        msg_pelican = PoseStamped()
        msg_pelican.pose.position.x = Xdesired[0]
        msg_pelican.pose.position.y = Xdesired[1]
        msg_pelican.pose.position.z = Xdesired[2]
        msg_pelican.pose.orientation.x = 0; msg_pelican.pose.orientation.y = 0 
        msg_pelican.pose.orientation.z = 0; msg_pelican.pose.orientation.w = 1
        self.pub_pelican.publish(msg_pelican); #time.sleep(0.1)
        

        self.counter = self.counter+1
        
        
        
        """
        #print PP, VV
        points=Scatter3d(mode = 'markers', name = '', x = PP[:,0], y = PP[:,1], z = PP[:,2], \
        marker = Marker( size=5, color='rgb(255, 0, 0)', symbol = 'cross')) #'#458B00'
        #simplexes = Mesh3d(alphahull = 10, name = '', x = PP[:,0], y = PP[:,1], z = PP[:,2], color='b', opacity=0.15)
        
        # first sphere
        u1, v1 = np.mgrid[0:2*np.pi:40j, 0:np.pi:10j]
        x1 = self.q1[0] + self.R1 * np.cos(u1)*np.sin(v1)
        y1 = self.q1[1] + self.R1 * np.sin(u1)*np.sin(v1)
        z1 = self.q1[2] + self.R1 * np.cos(v1)
        sp1 = Surface(x=x1, y=y1, z=z1, colorscale='Jet')    

        
        fig = plt.figure(figsize=(16,16))
        ax = fig.gca(projection='3d')
        # second sphere
        u2, v2 = np.mgrid[0:2*np.pi:40j, 0:np.pi:10j]
        x2 = self.q2[0] + self.R2 * np.cos(u2)*np.sin(v2)
        y2 = self.q2[1] + self.R2 * np.sin(u2)*np.sin(v2)
        z2 = self.q2[2] + self.R2 * np.cos(v2)
        sp2 = Surface(x=x2, y=y2, z=z2, colorscale='Jet')
        #ax.plot_wireframe(x2, y2, z2, color="r") 
        
        # third sphere
        u3, v3 = np.mgrid[0:2*np.pi:40j, 0:np.pi:10j]
        x3 = self.q3[0] + self.R3 * np.cos(u3)*np.sin(v3)
        y3 = self.q3[1] + self.R3 * np.sin(u3)*np.sin(v3)
        z3 = self.q3[2] + self.R3 * np.cos(v3)
        #ax.plot_wireframe(x3, y3, z3, color="r") 
        sp3 = Surface(x=x3, y=y3, z=z3, colorscale='Jet' )
        
        # fourth sphere
        u4, v4 = np.mgrid[0:2*np.pi:40j, 0:np.pi:10j]
        x4 = self.q4[0] + self.R4 * np.cos(u4)*np.sin(v4)
        y4 = self.q4[1] + self.R4 * np.sin(u4)*np.sin(v4)
        z4 = self.q4[2] + self.R4 * np.cos(v4)
        #ax.plot_wireframe(x4, y4, z4, color="r") 
        sp4 = Surface(x=x4, y=y4, z=z4, colorscale='Jet')

        # boundary sphere
        u0, v0 = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x0 = self.q0[0] + self.R0 * np.cos(u0)*np.sin(v0)
        y0 = self.q0[1] + self.R0 * np.sin(u0)*np.sin(v0)
        z0 = self.q0[2] + self.R0 * np.cos(v0)
        #ax.plot_wireframe(x0, y0, z0, color="r") 
        spb = Surface(x=x0, y=y0, z=z0, colorscale='Jet', opacity = 0.4)        

        ud, vd = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        xd = self.qd[0] + self.Rd * np.cos(ud)*np.sin(vd)
        yd = self.qd[1] + self.Rd * np.sin(ud)*np.sin(vd)
        zd = self.qd[2] + self.Rd * np.cos(vd)
        #ax.plot_wireframe(x0, y0, z0, color="r") 
        spd = Surface(x=xd, y=yd, z=zd, colorscale='Blues', opacity = 0.9) 
        
        fig=Figure(data=Data([points, sp1, sp2, sp3, sp4, spb, spd]))
        offline.plot(fig)
        
        ax.quiver(PP[:,0], PP[:,1], PP[:,2],VV[:,0], VV[:,1], VV[:,2])
        ax.set_xlabel('$X$', fontsize= 20)
        ax.set_ylabel('$Y$', fontsize = 20)
        ax.set_zlabel('$Z$', fontsize = 20)
        plt.savefig("trajectory_@k={:02}.jpeg".format(self.k)); plt.close()
        """
       

if __name__ == '__main__':
    rospy.init_node('NavigationFunction_Trajectory', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    start_time = time.time()
    try:
        while not rospy.is_shutdown(): 
            current_time = time.time()
            t = current_time-start_time
            traj = NavigationFunction_Trajectory(t)
            #traj.plotting()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass

