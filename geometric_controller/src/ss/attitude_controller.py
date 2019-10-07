#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  12 13:30:01 2017
This is the main controller as proposed by T Lee in his paper: 
"Geomentric tracking control of a quadrotor in SE(3)"
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'


import rospy
import time
import numpy as np
import scipy.linalg 
#from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
#from tf import TransformListener, TransformerROS
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion 
from parameters import parameters
#from isy_geometric_controller.msg import Desired_Trajectory, control_inputs
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators

class attitude_controller(object): 
    def __init__(self, name_of_uav,parameters): 
        #self.time = tt
        self.uav = name_of_uav
        self.m = parameters[0]
        self.J = parameters[1]
        self.kx = parameters[2]
        self.kv = parameters[3]
        self.kR = parameters[4]
        self.g = parameters[5]
        self.kOmega = parameters[6]
        self.d = parameters[7]
        self.tau_m = parameters[8]
        self.tau_t = parameters[9]
        self.e = parameters[10]
        self.counter = 0
        self.kR_normalized = np.dot(np.linalg.inv(self.J), self.kR)
        self.kOmega_normalized = np.dot(np.linalg.inv(self.J), self.kOmega)
        self.z = 3 # highet at which the quad is supposed to hover during attitude mode 
        self.pub = rospy.Publisher('/hummingbird/command/motor_speed', Actuators, queue_size = 1)
        try:
            self.time = time.time()
            odom = rospy.Subscriber('/hummingbird/odometry_sensor1/odometry', Odometry, self.callback)
            #traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory)
            #accel = message_filters.Subscriber('/pelican/imu', Imu)
            #ts = message_filters.TimeSynchronizer([odom, traj, accel], 1)
            #ts = message_filters.TimeSynchronizer([odom, accel], 1)
            
            #ts.registerCallback(self.callback)
            rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
   
    #def callback(self, odom, traj, accel): 
    def callback(self, odom):
        t = time.time()-self.time
        
        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]]);
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        V = np.dot(R, _V)
        Vd = ar([[0], [0], [0]])
        ad = ar([[0], [0], [0]])
        #acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
        vel= open('velocty.txt', 'a')
        vel.write("v:%s,%s,%s\n" % (V[0][0],V[1][0],V[2][0]))
        #pos= open('position.txt', 'a')
        #pos.write("x:%s,%s,%s\n" % (X[0][0],X[1][0],X[2][0]))        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0],0,-Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])
        
        # specify desired rotation matrix
        # to start with as per the lee's paper: Rd = exp[2pi(t-4)e_1_hat]
        
        if t>0.5 and t<=1.1666666:        
            e2 = tp(self.e[:,1][np.newaxis])
            e2_hat = ar([[0,-e2[2][0], e2[1][0]], [e2[2][0],0,-e2[0][0]], [-e2[1][0], e2[0][0], 0]])
            a = 3*np.pi
            Rc = scipy.linalg.expm(a*(t-0.5)*e2_hat)
            #Rc = scipy.linalg.expm(2*np.pi*(self.time-4)*e2_hat)
            Rc_dot = a*np.dot(e2_hat, Rc)
            Rc_ddot = a**2*np.dot(np.linalg.matrix_power(e2_hat,2), Rc)
        else: 
            Rc = self.e
            Rc_dot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3)))))
            Rc_ddot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3)))))
        """
        Rc = self.e
        Rc_dot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3)))))
        Rc_ddot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3)))))
        """   
        Rotation = open('desired_Rotation.txt', 'a')
        Rotation.write("%s:%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (t,Rc[0][0],Rc[1][0],Rc[2][0],Rc[0][1],Rc[1][1],Rc[2][1],Rc[0][2],Rc[1][2],Rc[2][2]))
        #print Rc
        Rot_cur = open('Current_Rotation.txt', 'a')
        Rot_cur.write("%s:%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (t,R[0][0],R[1][0],R[2][0],R[0][1],R[1][1],R[2][1],R[0][2],R[1][2],R[2][2]))
        
        Omega_c_hat = np.dot(Rc.transpose(), Rc_dot)
        Omega_c = ar([[-Omega_c_hat[1][2]], [Omega_c_hat[0][2]], [-Omega_c_hat[0][1]]])
        Omega_c_dot_hat = np.dot(tp(Rc),Rc_ddot) + np.dot(Omega_c_hat.transpose(),Omega_c_hat)         
        Omega_c_dot = ar([[-Omega_c_dot_hat[1][2]], [Omega_c_dot_hat[0][2]], [-Omega_c_dot_hat[0][1]]])
        
        
        """
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
        pos= open('position.txt', 'a')
        pos.write("x:%s,%s,%s,xd:%s,%s,%s\n" % (X[0][0],X[1][0],X[2][0], Xd[0][0],Xd[1][0],Xd[2][0]))  
        Vd = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        ad = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        b1d = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])

        b1d_dot = ar([[traj.desired_direction_dot.x], [traj.desired_direction_dot.y], [traj.desired_direction_dot.z]])
        b1d_ddot = ar([[traj.desired_direction_ddot.x], [traj.desired_direction_ddot.y], [traj.desired_direction_ddot.z]])

        correction = np.array([[0],[0],[0.1]])
        ex = X-Xd-correction 
        ev = V-Vd
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*tp(self.e[:,2][np.newaxis]) + ad # desired direction 
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        #b1c = -tp(np.cross(tp(b3c), np.cross(tp(b3c),tp(b1d))))/np.linalg.norm(np.cross(tp(b3c), tp(b1d))) 
        #b2c = tp(np.cross(tp(b3c), tp(b1c)))
        b2c = tp(np.cross(tp(b3c),tp(b1d))/np.linalg.norm(np.cross(tp(b3c), tp(b1d)))) # vector b2d 
        b1c = tp(np.cross(tp(b2c), tp(b3c)))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix
        """
        """
        if self.counter != 0:
            delt = time.time()-self.previous_time
            Rc_dot = (Rc-self.Rc_)/delt 
            Rc_ddot = (Rc_dot-self.Rc_dot_)/delt
            self.Rc_ = Rc; self.Rc_dot_ = Rc_dot
            self.previous_time = time.time()
        else: 
            Rc_dot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3))))); self.Rc_ = Rc
            Rc_ddot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3))))); self.Rc_dot_ = Rc_dot
            self.previous_time = time.time() 
        
        if self.counter != 0:
            delt = time.time()-self.previous_time
            ad_dot = (ad-self._ad)/delt; ad_ddot = (ad_dot-self._ad_dot)/delt   
            V_ddot = (acc-self.acc_)/delt
            self._ad = ad; self.acc_ = acc; self._ad_dot = ad_dot
            self.previous_time = time.time()
        else: 
            ad_dot = tp(np.zeros((1,3))); self._ad = ad
            V_ddot = tp(np.zeros((1,3))); self.acc_ = acc; ad_ddot = tp(np.zeros((1,3))); self._ad_dot = ad_dot
            self.previous_time = time.time()  
            
        
        A = mult(self.kx, ex) + mult(self.kv, ev)/self.m - self.g*tp(self.e[:,2][np.newaxis]) - ad
        ex_dot =  ev ; ev_dot = acc-ad
        A_dot = mult(self.kx, ex_dot) + mult(self.kv, ev_dot)/self.m - ad_dot # calculate desired direction 
        b3c_dot = -A_dot/np.linalg.norm(A) + (np.dot(tp(A),A_dot)/np.linalg.norm(A)**3)*A
        
        C = tp(np.cross(tp(b3c),tp(b1d)))
        C_dot = tp(np.cross(tp(b3c_dot), tp(b1d)) + np.cross(tp(b3c), tp(b1d_dot)))
        b2c_dot = C_dot/np.linalg.norm(C) - (np.dot(tp(C),C_dot)/np.linalg.norm(C)**3)*C
        b1c_dot = tp(np.cross(tp(b2c_dot), tp(b3c)) + np.cross(tp(b2c), tp(b3c_dot)))
        
        Rc_dot = np.column_stack((b1c_dot, b2c_dot, b3c_dot)) # desired rotation matrix derivative
        #np.putmask(Rc_dot, abs(Rc_dot)<1e-9,0)
        Omega_c_hat = np.dot(tp(Rc),Rc_dot) # skew-symmetric desired angular velocity 
        Omega_c = ar([[-Omega_c_hat[1][2]], [Omega_c_hat[0][2]], [-Omega_c_hat[0][1]]])
        #np.putmask(Omega_c_hat, abs(Omega_c_hat)<1e-9,0)
        
        ex_ddot = ev_dot; ev_ddot = V_ddot-ad_dot
        A_ddot = mult(self.kx, ex_ddot) + mult(self.kv, ev_ddot)/self.m - ad_ddot

        b3c_ddot = -A_ddot/np.linalg.norm(A) + 2*(np.dot(tp(A),A_dot)/np.linalg.norm(A)**3)*A_dot +\
        (np.linalg.norm(A_dot)**2+np.dot(tp(A),A_ddot))*A/np.linalg.norm(A)**3 - 3*(np.dot(tp(A),A_dot)**2/np.linalg.norm(A)**5)*A
        
        C_ddot = tp(np.cross(tp(b3c_ddot), tp(b1d)) + 2*np.cross(tp(b3c_dot), tp(b1d_dot)) + np.cross(tp(b3c), tp(b1d_ddot)))
        b2c_ddot = C_ddot/np.linalg.norm(C) - 2*(np.dot(tp(C),C_dot)/np.linalg.norm(C)**3)*C_dot -\
        (np.linalg.norm(C_dot)**2+np.dot(tp(C),C_ddot))*C/np.linalg.norm(C)**3 + 3*(np.dot(tp(C),C_dot)**2/np.linalg.norm(C)**5)*C
        
        #b2c_ddot= tp(np.cross(tp(b3c_ddot), tp(b1d)) + 2*np.cross(tp(b3c_dot), tp(b1d_dot))+np.cross(tp(b3c), tp(b1d_ddot)))
        b1c_ddot= tp(np.cross(tp(b2c_ddot), tp(b3c)) + 2*np.cross(tp(b2c_dot), tp(b3c_dot))+np.cross(tp(b2c), tp(b3c_ddot)))

        Rc_ddot = np.column_stack((b1c_ddot, b2c_ddot, b3c_ddot)) # desired rotation matrix derivative
        
        
        Omega_c_hat = np.dot(tp(Rc),Rc_dot) # skew-symmetric desired angular velocity 
        Omega_c = ar([[-Omega_c_hat[1][2]], [Omega_c_hat[0][2]], [-Omega_c_hat[0][1]]])
        Omega_c_dot_hat = np.dot(tp(Rc),Rc_ddot) + np.dot(Omega_c_hat.transpose(),Omega_c_hat) 
        
        Omega_c_dot = ar([[-Omega_c_dot_hat[1][2]], [Omega_c_dot_hat[0][2]], [-Omega_c_dot_hat[0][1]]])
        """
        eR_hat = 0.5*(np.dot(tp(Rc), R) - np.dot(tp(R), Rc)) # eR skew symmetric matrix	
        eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        eOmega = Omega - np.dot(np.dot(tp(R), Rc), Omega_c) # vector that gives error in angular velocity
       
        Z = np.dot(np.dot(Omega_hat.dot(tp(R)),Rc),Omega_c) - np.dot(np.dot(tp(R),Rc), Omega_c_dot) 
        #self.M1 = -1.0*(mult(self.kR_normalized, eR) + mult(self.kOmega_normalized, eOmega)) + \
        #tp(np.cross(tp(Omega), tp(Omega))) - Z   
        #self.M = np.dot(self.J,self.M1)
        
        self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega)))) - np.dot(self.J,Z)
        
        _f = -self.kx[2][0]*(X[2][0]-self.z)-self.kv[2][0]*(V[2][0]-Vd[2][0])+self.m*(self.g*tp(self.e[:,2][np.newaxis]) + ad[2][0])
        self.f = np.dot(tp(_f), tp(R[:,2][np.newaxis]))

        T = tp(ar([[self.M[0][0],self.M[1][0],self.M[2][0],self.f[0][0]]]))
        c1 = tp(ar([[0, -self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c2 = tp(ar([[self.d*self.tau_t, 0, -self.tau_t*self.tau_m, self.tau_t]]))
        c3 = tp(ar([[0, self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c4 = tp(ar([[-self.d*self.tau_t, 0, -self.tau_t*self.tau_m, self.tau_t]]))
        C = np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2
        w_square = np.dot(np.linalg.inv(C),T)
        w = np.sqrt(np.abs(w_square))
        
        Msg = Actuators()
        Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]
        self.pub.publish(Msg)

        #self.pub.publish(Msg)
        #rospy.sleep(0.1)
        self.counter+=1

if __name__ == '__main__':
    rospy.init_node('attitude_controller', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    p = parameters("hummingbird")
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]
    #start_time = time.time()
    try: 
        while not rospy.is_shutdown(): 
            current_time = time.time()
            #t = current_time-start_time
            c = attitude_controller("pelican", parameter_list)
            #rospy.spin()
            r.sleep()
    except rospy.ROSInterruptException(): 
        pass

