#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 30 16:54:20 2019
my expriment with unscented kalman filter 
@author: indrajeet
"""

import numpy as np 
from math import cos, sin
import rospy
from matplotlib import pyplot as plt 
import time, operator, scipy, math

#from functools import reduce
#from itertools import chain
#from scipy.linalg import block_diag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion as Q
import tf.transformations
import scipy, scipy.linalg



class fast_propagate(object): 
    """
    ROS node that propagates the estimates at 62 Hz
    """
    
    def __init__(self, name, number, freq):
        """initialize the parameters and subscribes to different topics"""
        self.path = '/home/pelican/data_collection/rhp_logs/'
        self.counter = 0 # a counter in imu callback
        #self.uav = name
	no = str(number)
        #imu_sampling_rate = 62 # eventually take it as ros parameter from launch file
        self.dt  = 1.0/freq
        # parameters for px4 imu, take all these from launch file
        self.gyro_noise = 1e-7
        self.accel_noise = 1e-7
        self.gyro_bias = 1e-6
        self.accel_bias = 1e-6
        self.start = time.time()

        self.gyro_bias_vector = np.ones(3)* self.gyro_bias*np.sqrt(self.dt)
        self.accel_bias_vector = np.ones(3)* self.accel_bias*np.sqrt(self.dt)
        self.g = np.array([[0], [0], [9.81]]) # gravity vector
        self.nsamples_smoothing = 5
        self.w = []; self.a = []
        self.VV = []

        self.fast_propagation = False
        self.pub = rospy.Publisher('/udrone1/fastodom', Odometry, queue_size = 1, tcp_nodelay=True)
        try:
            rospy.Subscriber('/'+name+no+'/ov_msckf/odomimu', Odometry, self.odomcallback, tcp_nodelay = True)
            rospy.Subscriber('/'+name+no+'/t265/imu', Imu, self.imucallback, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    def rotmat2quat_new(self, RR): 
        """function to convert a given rotation matrix to a quaternion """ 
        trace = np.trace(RR)
        q = np.zeros(4)
        if RR[0][0] > trace and RR[0][0] > RR[1][1] and RR[0][0] > RR[2][2]:
            q[0] = np.sqrt((1 + (2 * RR[0][0]) - trace) / 4)
            q[1] = (1 / (4 * q[0])) * (RR[0][1] + RR[1][0])
            q[2] = (1 / (4 * q[0])) * (RR[0][2] + RR[2][0])
            q[3] = (1 / (4 * q[0])) * (RR[1][2] - RR[2][1])

        elif RR[1][1] >= trace and RR[1][1] >= RR[0][0] and RR[1][1] >= RR[2][2]:
            q[1] = np.sqrt((1 + (2 * RR[1][1]) - trace) / 4)
            q[0] = (1 / (4 * q[1])) * (RR[0][1] + RR[1][0])
            q[2] = (1 / (4 * q[1])) * (RR[1][2] + RR[2][1])
            q[3] = (1 / (4 * q[1])) * (RR[2][0] - RR[0][2])
            
        elif RR[2][2] >= trace and RR[2][2] >= RR[0][0] and RR[2][2] >= RR[1][1]:
            q[2] = np.sqrt((1 + (2 * RR[2][2]) - trace) / 4)
            q[0] = (1 / (4 * q[2])) * (RR[0][2] + RR[2][0])
            q[1] = (1 / (4 * q[2])) * (RR[1][2] + RR[2][1])
            q[3] = (1 / (4 * q[2])) * (RR[0][1] - RR[1][0])
        else:
            q[3] = np.sqrt((1 + trace) / 4)
            q[0] = (1 / (4 * q[3])) * (RR[1][2] - RR[2][1])
            q[1] = (1 / (4 * q[3])) * (RR[2][0] - RR[0][2])
            q[2] = (1 / (4 * q[3])) * (RR[0][1] - RR[1][0])
        if q[3] < 0: 
            q = -q
        q = q/np.linalg.norm(q)
 
        return Q(q[3], q[0], q[1], q[2])

    def rotmat2quat(self, RR): 
        """function to convert a given rotation matrix to a quaternion """ 
        trace = np.trace(RR)
        if trace >= 0:
            S = 2.0 * np.sqrt(trace + 1)
            qw = 0.25 * S
            qx = (RR[2][1] - RR[1][2]) / S
            qy = (RR[0][2] - RR[2][0]) / S
            qz = (RR[1][0] - RR[0][1]) / S
        elif RR[0][0] > RR[1][1] and RR[0][0] > RR[2][2]: 
            S = 2.0 * np.sqrt(1 + RR[0][0] - RR[1][1] - RR[2][2])
            qw = (RR[2][1] - RR[1][2]) / S
            qx = 0.25 * S
            qy = (RR[1][0] + RR[0][1]) / S
            qz = (RR[0][2] + RR[2][0]) / S
        elif RR[1][1] > RR[2][2]:
            S = 2.0 * np.sqrt(1 + RR[1][1] - RR[0][0] - RR[2][2])
            qw = (RR[0][2] - RR[2][0]) / S
            qx = (RR[1][0] + RR[0][1]) / S
            qy = 0.25 * S
            qz = (RR[2][1] + RR[1][2]) / S
        else:
            S = 2.0 * np.sqrt(1 + RR[2][2] - RR[0][0] - RR[1][1])            
            qw = (RR[1][0] - RR[0][1]) / S
            qx = (RR[0][2] + RR[2][0]) / S            
            qy = (RR[2][1] + RR[1][2]) / S   
            qz = 0.25 * S
        if qw < 0: 
            qw =-qw; qx = -qx; qy = -qy; qz = -qz
 
        return Q(qw, qx, qy, qz)



    def odomcallback(self, data): 
        """"callback that takes the state estimates from open_vins, all these values are measurements to the UKF
        position velocity are of CoG in world frame, orientation"""

        px = data.pose.pose.position.x; py = data.pose.pose.position.y; pz = data.pose.pose.position.z
        self.X = np.array([[px], [py], [pz]])
        vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z
        self.V = np.array([[vx], [vy], [vz]])
        q = Q(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        self.R = q.rotation_matrix

        self.fast_propagation = True
        
       
    def imucallback(self, data):
        """fast propagation"""
        if self.fast_propagation == True: 
            
            w_meas = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
            a_meas = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
            
            self.gyro_mean = np.array([np.random.normal(0,1), np.random.normal(0,1), np.random.normal(0,1)]) * self.gyro_noise / np.sqrt(self.dt)
            self.accel_mean = np.array([np.random.normal(0,1), np.random.normal(0,1), np.random.normal(0,1)]) * self.accel_noise / np.sqrt(self.dt)
            
            w = w_meas - self.gyro_bias_vector + self.gyro_mean
            a = a_meas - self.accel_bias_vector + self.accel_mean 

            #-----smoothing w and a to reduce error in fast propagated velocity-----------
            self.w.append(w); self.a.append(a)
            w = np.mean(self.w, axis = 0); a = np.mean(self.a, axis = 0) 
            if len(self.w) >= self.nsamples_smoothing:
                self.w.pop(0); self.a.pop(0)
            #elif len(self.w) == self.nsamples_smoothing: 
            #    w = np.mean(self.w, axis = 0); a = np.mean(self.a, axis = 0) 
            #-----smoothing w and a to reduce error in fast propagated velocity-----------
    
            w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
            
            self.X = self.X + self.V * self.dt 
            self.V = self.V + self.dt * (np.dot(self.R, a[np.newaxis].T)-self.g)
            self.R = np.dot(self.R, (np.identity(3) + w_hat*self.dt))


            #-----smoothing V to reduce error in fast propagation -----------
            #self.VV.append(self.V)
            #self.V = np.mean(self.VV, axis = 0)
            #if len(self.VV) >= self.nsamples_smoothing:
            #    self.VV.pop(0)                
            #-----smoothing V to reduce error in fast propagation -----------
                        
            q = self.rotmat2quat(self.R)
     
            msg = Odometry()
            msg.header.frame_id = 'global'
            #msg.child_frame_id = 'mav_cog'
            msg.header.stamp = data.header.stamp
            msg.pose.pose.position.x = self.X[0][0]; msg.pose.pose.position.y = self.X[1][0]; msg.pose.pose.position.z = self.X[2][0]
            msg.pose.pose.orientation.x = q[1]; msg.pose.pose.orientation.y = q[2]; msg.pose.pose.orientation.z = q[3]; msg.pose.pose.orientation.w = q[0]
            msg.twist.twist.linear.x = self.V[0][0] ; msg.twist.twist.linear.y = self.V[1][0]; msg.twist.twist.linear.z = self.V[2][0]      

            self.pub.publish(msg)

if __name__ == '__main__':
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    freq = rospy.get_param('freq')
    rospy.init_node('fast_odometry_propagation', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(62)
    traj = fast_propagate(name, number, freq)
    try: 
        while not rospy.is_shutdown():            
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                
