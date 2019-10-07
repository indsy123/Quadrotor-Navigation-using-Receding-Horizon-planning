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



class unscented_kalman_filter(object): 
    """
    ROS node that runs a Unscented kalman Filter.
    The state vector is [position, velocity, roll, pitch, yaw, gyro_bias, accel_bias] as in paper 
    "Fast autonomous flight in GPS defined environments, K Mohta at el"
    """
    
    def __init__(self, name_of_uav):
        """initialize the parameters and subscribes to different topics"""
        self.counter = 0 # a counter in imu callback
        self.uav = name_of_uav
        imu_sampling_rate = 50 # eventually take it as ros parameter from launch file
        self.dt  = 1.0/imu_sampling_rate
        # parameters for px4 imu, take all these from launch file
        self.gyro_noise = 1e-3
        self.accel_noise = 1e-3
        self.gyro_bias = 1e-4
        self.accel_bias = 1e-4
        
        initial_state_cov  = 0.5e-1
        # initialize covariance matrices for now later take it form Patrick's code?? or not??
        # self.C is used for covariance
        self.x_cov = self.v_cov = self.rpy_cov = np.identity(3)*initial_state_cov**2 # position, velocity, rpy
        self.gyro_cov = np.identity(3)*self.gyro_noise**2; self.accel_cov = np.identity(3)*self.accel_noise**2 # gyro, accelerometer
        
        self.gyro_mean = self.accel_mean = np.zeros(3) # gyro and accel noise is 0 mean 
        self.gyro_bias_vector = np.ones(3)* self.gyro_bias
        self.accel_bias_vector = np.ones(3)* self.accel_bias
        
        self.process_mean_noise = np.zeros(12)
        self.process_Q = np.identity(12)
        self.process_Q[0:3]  = self.gyro_noise**2 * self.process_Q[0:3]
        self.process_Q[3:6]  = self.accel_noise**2 * self.process_Q[3:6]
        self.process_Q[6:9]  = self.gyro_bias**2 * self.process_Q[6:9]
        self.process_Q[9:12]  = self.accel_bias**2 * self.process_Q[9:12]
        
        self.X = np.array([0,0,0,0,0,0,0,0,0])
        self.X = np.concatenate((self.X, self.gyro_bias_vector, self.accel_bias_vector))
        self.Pk = scipy.linalg.block_diag(self.x_cov, self.v_cov, self.rpy_cov, self.gyro_cov, self.accel_cov)
        self.Nstates = 15 # 9 states and 6 biases
        self.Nprocess = 12 # process, 6 for gyro noises and 6 for bias noises?? really
        self.Nmeasurement = 9

        self.alpha = 0.01; self.beta = 2.0; self.kappa = 0.0
        self.g = np.array([[0], [0], [9.81]]) # gravity vector
        self.meas_noise_density = 1e-1
        self.measurement_noise_cov = np.identity(9)* self.meas_noise_density**2
        self.Pxy = np.zeros
        
        self.pub = rospy.Publisher('/diy1/ukf/odometry', Odometry, queue_size = 1, tcp_nodelay=True)
        try:
            rospy.Subscriber('/ov_msckf/odomimu', Odometry, self.odomcallback, queue_size = 10, tcp_nodelay = True)
            rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imucallback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    def nearestPD(self, A):
        """Find the nearest positive-definite matrix to input
    
        A Python/Numpy port of John D'Errico's `nearestSPD` MATLAB code [1], which
        credits [2].
    
        [1] https://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd
    
        [2] N.J. Higham, "Computing a nearest symmetric positive semidefinite
        matrix" (1988): https://doi.org/10.1016/0024-3795(88)90223-6
        """
    
        B = (A + A.T) / 2
        _, s, V = np.linalg.svd(B)
    
        H = np.dot(V.T, np.dot(np.diag(s), V))
    
        A2 = (B + H) / 2
    
        A3 = (A2 + A2.T) / 2
    
        if self.isPD(A3):
            return A3
    
        spacing = np.spacing(np.linalg.norm(A))
        # The above is different from [1]. It appears that MATLAB's `chol` Cholesky
        # decomposition will accept matrixes with exactly 0-eigenvalue, whereas
        # Numpy's will not. So where [1] uses `eps(mineig)` (where `eps` is Matlab
        # for `np.spacing`), we use the above definition. CAVEAT: our `spacing`
        # will be much larger than [1]'s `eps(mineig)`, since `mineig` is usually on
        # the order of 1e-16, and `eps(1e-16)` is on the order of 1e-34, whereas
        # `spacing` will, for Gaussian random matrixes of small dimension, be on
        # othe order of 1e-16. In practice, both ways converge, as the unit test
        # below suggests.
        I = np.eye(A.shape[0])
        k = 1
        while not self.isPD(A3):
            mineig = np.min(np.real(np.linalg.eigvals(A3)))
            A3 += I * (-mineig * k**2 + spacing)
            k += 1
    
        return A3

    def isPD(self, B):
        """Returns true when input is positive-definite, via Cholesky"""
        try:
            _ = np.linalg.cholesky(B)
            return True
        except np.linalg.LinAlgError:
            return False



    
    def eular_to_rotation_matrix(self, rpy):
        """transform euler angles to rotation matrix. try not using it, python function call is slow"""
        r = rpy[0]; p = rpy[1]; y = rpy[2]
        Rx = np.array([[cos(r), -sin(r), 0], [sin(r), cos(r), 0], [0, 0, 1]])
        Ry = np.array([[cos(p), 0, sin(p)], [0, 1, 0], [-sin(p), 0, cos(p)]])
        Rz = np.array([[1, 0, 0], [0, cos(y), -sin(y)], [0, sin(y), cos(y)]])
        R = np.dot(Rz, np.dot(Ry, Rx))
        return R


    def calculate_sigmas_weights(self, Xa, Pa, n, p): 
        """prediction step of the filter"""
        self.L = n+p
        # initializing sigmas
        sigmas = np.zeros((2*self.L+1, self.L))
        #self.kappa = 3-self.L
        # defining weights
        _lambda = self.alpha**2 * (self.L+self.kappa) - self.L
        #_lambda = 8.0 
        Wc = np.full(2*self.L+1, 1/(2*(self.L + _lambda)))
        Wm = np.full(2*self.L+1, 1/(2*(self.L + _lambda)))
        Wc[0] = _lambda / (_lambda + self.L)#(_lambda / (self.L + _lambda)) + (1 - self.alpha **2 + self.beta)
        Wm[0] = _lambda / (_lambda + self.L)

        # calculating sigmas
        sigmas[0] = Xa
        #print 'Wm', Wm, 'sum', sum(Wm)
        #print 'Wc', Wc, 'sum', sum(Wc)
        U = scipy.linalg.cholesky((self.L + _lambda)*Pa)
        
        """
        Pa_isPD = self.isPD((self.L + _lambda)*Pa)
        if Pa_isPD == True: 
            U = scipy.linalg.cholesky((self.L + _lambda)*Pa)
        else: 
            Pa_PD = self.nearestPD((self.L + _lambda)*Pa)
            U = scipy.linalg.cholesky(Pa_PD)
        """
        
        for k in range(self.L): 
            sigmas[k+1] = Xa + U[k]            
            sigmas[self.L+k+1] = Xa - U[k]
        return sigmas, Wm, Wc
        
        
        

    def odomcallback(self, data): 
        """"callback that takes the state estimates from open_vins, all these values are measurements to the UKF
        position velocity are of CoG in world frame, orientation"""

        px = data.pose.pose.position.x; py = data.pose.pose.position.y; pz = data.pose.pose.position.z
        vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler_angles = tf.transformations.euler_from_quaternion (q, 'szyx')
        roll = euler_angles[0]; pitch = euler_angles[1]; yaw = euler_angles[2]  
        #print 'roll, pitch, yaw', roll, pitch, yaw
        
        self.Y = np.array([px, py, pz, vx, vy, vz, roll, pitch, yaw])   
        self.measurement_Q = scipy.linalg.block_diag(self.x_cov, self.v_cov, self.rpy_cov)
        
        Xa_k1_k = np.concatenate((self.x_k1_k, np.zeros(9)))
        
        Pa_k1_k = scipy.linalg.block_diag(self.Pk1_k, self.measurement_noise_cov)

        #print len(Xa_k1_k), Pa_k1_k.shape
        Ysigmas_k, Wm_k, Wc_k = self.calculate_sigmas_weights(Xa_k1_k, Pa_k1_k, self.Nstates, self.Nmeasurement)
        
        self.y_k1_k = [np.dot(Wm_k, Ysigmas_k[:,[i]])[0] for i in range(Ysigmas_k.shape[1])]
        #outer_product = [np.outer((Ysigmas_k[i][:15]-np.asarray(self.y_k1_k)[:15]), (Ysigmas_k[i][:15]-np.asarray(self.y_k1_k)[:15]).T) for i in range(Ysigmas_k.shape[0])]
        outer_product = [np.outer((Ysigmas_k[i][:9]-np.asarray(self.y_k1_k)[:9]), (Ysigmas_k[i][:9]-np.asarray(self.y_k1_k)[:9]).T) for i in range(Ysigmas_k.shape[0])]
        elements_of_Pyy = [Wc_k[i]*outer_product[i] for i in range(len(Wc_k))]
        Pyy = sum(elements_of_Pyy)  
        
        #outer_product = [np.outer((Ysigmas_k[i][:15]-np.asarray(self.x_k1_k)), (Ysigmas_k[i]-np.asarray(self.y_k1_k))[:15].T) for i in range(Ysigmas_k.shape[0])]
        outer_product = [np.outer((self.Xsigmas[i][:15]-np.asarray(self.x_k1_k)), (Ysigmas_k[i]-np.asarray(self.y_k1_k))[:9].T) for i in range(Ysigmas_k.shape[0])]
        elements_of_Pxy = [Wc_k[i]*outer_product[i] for i in range(Ysigmas_k.shape[0])]
        Pxy = sum(elements_of_Pxy)   
        
        K = np.dot(Pxy, np.linalg.inv(Pyy))

        self.X = self.X + np.dot(K, (self.Y-self.y_k1_k[:9])[np.newaxis].T).ravel()
        self.Pk = self.Pk1_k[0:self.Nstates, 0:self.Nstates] - np.dot(K, np.dot(Pyy, K.T))
        
        #print self.Pk
        print 'eigan values', np.linalg.eigvals(self.Pk)
        #print 'first term', self.Pk1_k[0:self.Nstates, 0:self.Nstates]
        #print 'second term', np.dot(K, np.dot(Pyy, K.T))


        
    def imucallback(self, data):
        """callback that take the IMU data and runs UKF to publish the open_vins estimates at the frequency of the IMU"""

        w_meas = np.array([data.angular_velocity.x, data.angular_velocity.z, data.angular_velocity.z])
        a_meas = np.array([data.linear_acceleration.x, data.linear_acceleration.z, data.linear_acceleration.z])
        w = w_meas - self.gyro_bias_vector + self.gyro_mean
        a = a_meas - self.accel_bias_vector + self.accel_mean 
        w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        
        self.gyro_mean = np.array([np.random.normal(0,1), np.random.normal(0,1), np.random.normal(0,1)]) * self.gyro_noise / np.sqrt(self.dt)
        self.accel_mean = np.array([np.random.normal(0,1), np.random.normal(0,1), np.random.normal(0,1)]) * self.accel_noise / np.sqrt(self.dt)
        
        # augument states        
        Xa_k = np.concatenate((self.X, self.process_mean_noise))
        #Qk = scipy.linalg.block_diag(self.gyro_cov, self.accel_cov)
        Pa_k = scipy.linalg.block_diag(self.Pk, self.process_Q)
        # calculate sigmas

        Xsigmas_k, Wm_k, Wc_k = self.calculate_sigmas_weights(Xa_k, Pa_k, self.Nstates, self.Nprocess)
        self.Xsigmas = Xsigmas_k

        # prediction step
        R_k = [self.eular_to_rotation_matrix(Xsigmas_k[i][6:9]) for i in range(Xsigmas_k.shape[0])]
        R_k1 = [np.dot(R_k[i], (np.identity(3) + w_hat*self.dt)) for i in range(Xsigmas_k.shape[0])]
        rpy_k1 = [tf.transformations.euler_from_matrix(R_k1[i]) for i in range(Xsigmas_k.shape[0])]
        v_k1 = [Xsigmas_k[i][3:6] + self.dt * (np.dot(R_k[i], a[np.newaxis].T)-self.g).ravel() for i in range(Xsigmas_k.shape[0])]
        p_k1 = [Xsigmas_k[i][0:3] + v_k1[i] * self.dt for i in range(Xsigmas_k.shape[0])]
        ba_k1 = [Xsigmas_k[i][9:12] + self.dt * Xsigmas_k[i][21:24] for i in range(Xsigmas_k.shape[0])]
        bw_k1 = [Xsigmas_k[i][12:15] + self.dt * Xsigmas_k[i][24:27] for i in range(Xsigmas_k.shape[0])]

      
        Xsigmas_k1 = [np.concatenate((p_k1[i], v_k1[i], rpy_k1[i], ba_k1[i], bw_k1[i], Xsigmas_k[i][15:27])) for i in range(Xsigmas_k.shape[0])]
        Xsigmas_k1 = np.asarray(Xsigmas_k1)
        #predicted mean and covariance
        self.x_k1_k = [np.dot(Wm_k, Xsigmas_k1[:,[i]])[0] for i in range(Xsigmas_k1.shape[1])]
        outer_product = [np.outer((Xsigmas_k1[i]-np.asarray(self.x_k1_k)), (Xsigmas_k1[i]-np.asarray(self.x_k1_k)).T) for i in range(Xsigmas_k1.shape[0])]
        elements_of_Pk1_k = [Wc_k[i]*outer_product[i] for i in range(len(Wc_k))]
        self.Pk1_k = sum(elements_of_Pk1_k)
        
        self.x_k1_k = self.x_k1_k[:self.Nstates]
        self.Pk1_k = self.Pk1_k[0:self.Nstates, 0:self.Nstates]
        self.gyro_bias_vector = self.x_k1_k[9:12]
        self.accel_bias_vector = self.x_k1_k[12:15]
        print self.X
        print self.x_k1_k
        print 'eigan values', np.linalg.eigvals(self.Pk1_k)
        #print 'rpy', self.x_k1_k[6], self.x_k1_k[7], self.x_k1_k[8]
        #K = np.dot(self.Pxy, np.linalg.inv(self.Pyy))
        #self.X = self.X + np.dot(K, (self.Y-self.y_k1_k[:15])[np.newaxis].T).ravel()
        #self.Pk = self.Pk1_k[0:self.Nstates, 0:self.Nstates] - np.dot(K, np.dot(self.Pyy, K.T))
        
        msg = Odometry()
        msg.header.stamp = data.header.stamp
        
        msg.pose.pose.position.x = self.x_k1_k[0]; msg.pose.pose.position.y = self.x_k1_k[1]; msg.pose.pose.position.z = self.x_k1_k[2]
        q = tf.transformations.quaternion_from_euler(self.x_k1_k[6], self.x_k1_k[7], self.x_k1_k[8], 'szyx')
        msg.pose.pose.orientation.x = q[0]; msg.pose.pose.orientation.y = q[1]; msg.pose.pose.orientation.z = q[2];msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.x_k1_k[3]; msg.twist.twist.linear.y = self.x_k1_k[4]; msg.twist.twist.linear.z = self.x_k1_k[5]        
        
        #msg.pose.pose.position.x = self.X[0]; msg.pose.pose.position.y = self.X[1]; msg.pose.pose.position.z = self.X[2]
        #q = tf.transformations.quaternion_from_euler(self.X[6], self.X[7], self.X[8], 'szyx')
        #msg.pose.pose.orientation.x = q[0]; msg.pose.pose.orientation.y = q[1]; msg.pose.pose.orientation.z = q[2];msg.pose.pose.orientation.w = q[3]
        #msg.twist.twist.linear_velocity.x = self.X[3]; msg.twist.twist.linear_velocity.y = self.X[4]; msg.twist.twist.linear_velocity.z = self.X[5]
        self.pub.publish(msg)
            

        """ block used just for testing between the performance of for in a line and map() function, to be deleted eventually
        as there is no speed benifit
        R_k = map(lambda i: self.eular_to_rotation_matrix(i), sigmas_k[:,[6,7,8]])
        R_k1 = map(lambda i: np.dot(i, (np.identity(3) + self.hat(w)*self.dt)), R_k)
        rpy_k1 = map(lambda i: tf.transformations.euler_from_matrix(i), R_k1)
        v_k1 = map(lambda i, j: i + self.dt * (np.dot(j, a[np.newaxis].T)-self.g).ravel(),  sigmas_k[:,[3,4,5]], R_k)
        p_k1 = map(lambda i, j: i + j * self.dt , sigmas_k[:,[0,1,2]], v_k1)
        ba_k1 = map(lambda i, j: i + self.dt * j,  sigmas_k[:,[9,10,11]], sigmas_k[:,[21,22,23]])
        bw_k1 = map(lambda i, j: i + self.dt * j, sigmas_k[:,[12,13,14]], sigmas_k[:,[24,25,26]])
        print '31', time.time()-t
        sigmas_k1 = [np.concatenate((p_k1[i], v_k1[i], rpy_k1[i], ba_k1[i], bw_k1[i], sigmas_k[i][15:27])) for i in range(sigmas_k.shape[0])]
        sigmas_k1 = np.asarray(sigmas_k1)
        x_bar_k1_k = [np.dot(Wm_k, sigmas_k1[:,[i]])[0] for i in range(sigmas_k1.shape[1])]
        print '32', time.time()-t
        """
        


if __name__ == '__main__':
    #name = rospy.get_param('uav_name')
    name = 'diy2'
    rospy.init_node('unscented_kalman_filter', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = unscented_kalman_filter(name)
    try: 
        while not rospy.is_shutdown():            
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                