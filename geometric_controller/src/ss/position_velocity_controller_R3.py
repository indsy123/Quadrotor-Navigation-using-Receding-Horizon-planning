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
#from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
#from tf import TransformListener, TransformerROS
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion 
from parameters import parameters
from isy_geometric_controller.msg import Desired_Trajectory, control_inputs
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators
from isy_geometric_controller.msg import velocities_from_navigation_function
from scipy.interpolate import splrep, splev

class velocity_controller(object): 
    def __init__(self, name_of_uav,parameters): 
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
        self.time_points = []; self.time = time.time()
        self.smooth_vx = []; self.smooth_vy = []; self.smooth_vz = []
        #self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size = 1)
        self.pub = rospy.Publisher('/control_inputs', control_inputs, queue_size = 100)
        try:
            odom = message_filters.Subscriber('/'+self.uav+'/vi_sensor/ground_truth/odometry', Odometry, tcp_nodelay = True)
            traj = message_filters.Subscriber('/NavigationFunctionTrajectory', velocities_from_navigation_function, tcp_nodelay = True)
            #odom2 = message_filters.Subscriber('/'+self.uav+'/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)            
            #traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory, tcp_nodelay = True) 
            accel = message_filters.Subscriber('/'+self.uav+'/ground_truth/imu', Imu, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([odom, accel, traj], 100)
            ts.registerCallback(self.callback)
            rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
   
    def callback(self, odom, accel, traj): 
        #a = time.time()

        X = ar([[odom.pose.pose.position.x-0.1], [odom.pose.pose.position.y], [odom.pose.pose.position.z+0.03]]);
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
        #V1 = np.dot(R, _V); 

        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        #Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0],0,-Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])




        # next two matrices are position and orientation offset of the vi sensor imu with the center of gravity
        vi_pos_off_hat = ar([[0.0, 0.03, 0.0], [-0.03, 0.0, -0.1], [0.0, 0.1, 0.0]]) 
        vi_rot_off = ar([[0.9950042, 0.0, 0.0998334], [0.0, 1.0, 0.0], [-0.0998334, 0.0, 0.9950042]])
        #vi_rot_off = ar([[0.7071, 0.0, 0.7071], [0.0, 1.0, 0.0], [-0.7071, 0.0, 0.7071]])

        intermediate = np.dot(vi_pos_off_hat, vi_rot_off)
        V1 = np.dot(vi_rot_off,_V) + np.dot(intermediate, Omega)         
        Omega = np.dot(vi_rot_off, Omega)
        R = np.dot(R, vi_rot_off.T)
        
        Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0], 0, -Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])
        V1 = np.dot(R, V1)
        acc = np.dot(R, acc)


        #VV = ar([[odom2.twist.twist.linear.x], [odom2.twist.twist.linear.y], [odom2.twist.twist.linear.z]])
        #qq = Quaternion(odom2.pose.pose.orientation.w, odom2.pose.pose.orientation.x,\
        #                odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z)
        #RR = qq.rotation_matrix
        #Omega2 = ar([[odom2.twist.twist.angular.x], [odom2.twist.twist.angular.y], [odom2.twist.twist.angular.z]])
        #Omega2_hat = ar([[0,-Omega2[2][0], Omega2[1][0]], [Omega2[2][0], 0, -Omega2[0][0]], [-Omega2[1][0], Omega2[0][0], 0]])
        
        #VV= np.dot(RR, VV)
        #acc2 = np.dot(RR, acc)        
        #imu_off = np.array([[0],[0],[0.07999]]) # offset of imu
        #imu_off_hat = ar([[0,-imu_off[2][0], imu_off[1][0]], [imu_off[2][0],0,-imu_off[0][0]], [-imu_off[1][0], imu_off[0][0], 0]])

        #V1 = V1 + np.dot(imu_off_hat, Omega)        
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
        Vd = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        ad = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        ad_dot = ar([[traj.desired_jerk.x], [traj.desired_jerk.y], [traj.desired_jerk.z]])

        b1d = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])
        b1d_dot = ar([[0], [0], [0]])
        b1d_ddot = ar([[0], [0], [0]])
        
        current_time = time.time()
        t = current_time - self.time
        self.time_points.append(t)
        
        """
        self.smooth_vx.append(V1[0][0]); self.smooth_vy.append(V1[1][0]); self.smooth_vz.append(V1[2][0])
        
        if self.counter < 5:
           V = V1; acc = tp(np.zeros((1,3))); V_ddot = tp(np.zeros((1,3)))
        else: 
            t = list(np.linspace(min(self.time_points), max(self.time_points), self.counter+1))
            if self.counter > 100: # number of points used in making spline 
                t.pop(0); self.smooth_vx.pop(0); self.smooth_vy.pop(0); self.smooth_vz.pop(0)
                t = list(np.linspace(min(self.time_points), max(self.time_points), 101))
                    
            a = splrep(t, self.smooth_vx, k = 2, s = 2)
            b = splrep(t, self.smooth_vy, k = 2, s = 2)
            c = splrep(t, self.smooth_vz, k = 2, s = 2)
            _vx = splev(t, a); _vy = splev(t, b); _vz = splev(t, c)
            _ax = splev(t, a, der = 1); _ay = splev(t, b, der = 1); _az = splev(t, c, der = 1)
            _jx = splev(t, a, der = 2); _jy = splev(t, b, der = 2); _jz = splev(t, c, der = 2)
           
            V = ar([[_vx[-1]], [_vy[-1]], [_vz[-1]]])            
            acc = ar([[_ax[-1]], [_ay[-1]], [_az[-1]]])
            V_ddot = ar([[_jx[-1]], [_jy[-1]], [_jz[-1]]])


        if self.counter != 0:
            delt = (max(self.time_points)-min(self.time_points))/self.counter+1
            ad_ddot = (ad_dot-self._ad_dot)/delt; self._ad_dot = ad_dot; self.previous_time = time.time()
        else: 

            ad_ddot = tp(np.zeros((1,3))); self._ad_dot = ad_dot; self.previous_time = time.time()  
        """
        #correction = np.array([[0],[0],[0.1]])
        
        if traj.controller == 1: 
            ex = ar([[0],[0],[0]])
        else: 
            ex = X-Xd
        
        ev = V1-Vd
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction 
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix
        error  = 0.5 * np.trace(self.e-np.dot(Rc.T, R))
        current_time = time.time()
        t = current_time - self.time
        self.time_points.append(t)
        
        if self.counter != 0:
            delt = (max(self.time_points)-min(self.time_points))/self.counter+1
            #self.current_time = time.time()
            #delt = self.current_time-self.previous_time
            #ad_dot = (ad-self._ad)/delt; 
            ad_ddot = (ad_dot-self._ad_dot)/delt   
            V_ddot = (acc-self.acc_)/delt
            #self._ad = ad; 
            self.acc_ = acc; 
            self._ad_dot = ad_dot
            self.previous_time = time.time()
        else: 
            #ad_dot = tp(np.zeros((1,3))); 
            #self._ad = ad
            V_ddot = tp(np.zeros((1,3))); self.acc_ = acc; ad_ddot = tp(np.zeros((1,3))); self._ad_dot = ad_dot
            self.previous_time = time.time()  
            

        f0 = open('velocity.txt', 'a')
        f0.write("%s, %s, %s, %s, %s, %s\n" % (V1[0][0], V1[1][0], V1[2][0], Vd[0][0], Vd[1][0], Vd[2][0]))
        f1 = open('accel.txt', 'a')
        f1.write("%s, %s, %s, %s, %s, %s\n" % (acc[0][0], acc[1][0], acc[2][0], ad[0][0], ad[1][0], ad[2][0]))
        f2 = open('position.txt', 'a')
        f2.write("%s, %s, %s, %s, %s, %s\n" % (X[0][0], X[1][0], X[2][0], Xd[0][0], Xd[1][0], Xd[2][0]))
        f3 = open('jerk.txt', 'a')
        f3.write("%s, %s, %s, %s, %s, %s\n" % (V_ddot[0][0], V_ddot[1][0], V_ddot[2][0], ad_dot[0][0], ad_dot[1][0], ad_dot[2][0]))

        #f4 = open('vel_compare.txt', 'a')
        #f4.write("%s, %s, %s, %s, %s, %s\n" % (V1[0][0], V1[1][0], V1[2][0], VV[0][0], VV[1][0], VV[2][0]))
        
        #f5 = open('angvel_compare.txt', 'a')
        #f5.write("%s, %s, %s, %s, %s, %s\n" % (Omega[0][0], Omega[1][0], Omega[2][0], Omega2[0][0], Omega2[1][0], Omega2[2][0]))
        #f6 = open('acc_compare.txt', 'a')
        #f6.write("%s, %s, %s, %s, %s, %s\n" % (acc[0][0], acc[1][0], acc[2][0], acc2[0][0], acc2[1][0], acc2[2][0]))

        f7 = open('error.txt', 'a')
        f7.write("%s\n" % (error))        

        A = (mult(self.kx, ex) + mult(self.kv, ev))/self.m - self.g*self.e[:,2][np.newaxis].T - ad
        #ex_dot = ar([[0],[0],[0]])
        if traj.controller == 1: 
            ex_dot = ar([[0],[0],[0]])
        else: 
            ex_dot = ev
        #ex_dot =  ev 
        ev_dot = acc-ad
        A_dot = (mult(self.kx, ex_dot) + mult(self.kv, ev_dot))/self.m - ad_dot # calculate desired direction 
        b3c_dot = -A_dot/np.linalg.norm(A) + (np.dot(A.T,A_dot)/np.linalg.norm(A)**3)*A
        
        C = tp(np.cross(b3c.T,b1d.T))
        C_dot = tp(np.cross(b3c_dot.T, b1d.T) + np.cross(b3c.T, b1d_dot.T))
        b2c_dot = C_dot/np.linalg.norm(C) - (np.dot(C.T,C_dot)/np.linalg.norm(C)**3)*C
        b1c_dot = tp(np.cross(b2c_dot.T, b3c.T) + np.cross(b2c.T, b3c_dot.T))
        
        Rc_dot = np.column_stack((b1c_dot, b2c_dot, b3c_dot)) # desired rotation matrix derivative
        
        #Omega_c_hat = np.dot(tp(Rc),Rc_dot) # skew-symmetric desired angular velocity 
        Omega_c_hat = np.dot(Rc.T, Rc_dot)
        #Omega_c_hat = ar([[0,0,0], [0,0,0], [0, 0, 0]])
        Omega_c = ar([[-Omega_c_hat[1][2]], [Omega_c_hat[0][2]], [-Omega_c_hat[0][1]]])

        
#        ex_ddot = ar([[0],[0],[0]]) 
        if traj.controller == 1: 
            ex_ddot = ar([[0],[0],[0]])
        else: 
            ex_ddot = ev_dot

        ev_ddot = V_ddot-ad_dot
        A_ddot = (mult(self.kx, ex_ddot) + mult(self.kv, ev_ddot))/self.m - ad_ddot

        b3c_ddot = -A_ddot/np.linalg.norm(A) + 2*(np.dot(A.T,A_dot)/np.linalg.norm(A)**3)*A_dot +\
        (np.linalg.norm(A_dot)**2+np.dot(A.T,A_ddot))*A/np.linalg.norm(A)**3 - 3*(np.dot(A.T,A_dot)**2/np.linalg.norm(A)**5)*A
        
        C_ddot = tp(np.cross(b3c_ddot.T, b1d.T) + 2*np.cross(b3c_dot.T, b1d_dot.T) + np.cross(b3c.T, b1d_ddot.T))
        b2c_ddot = C_ddot/np.linalg.norm(C) - 2*(np.dot(C.T,C_dot)/np.linalg.norm(C)**3)*C_dot -\
        (np.linalg.norm(C_dot)**2+np.dot(C.T,C_ddot))*C/np.linalg.norm(C)**3 + 3*(np.dot(C.T,C_dot)**2/np.linalg.norm(C)**5)*C

        b1c_ddot= tp(np.cross(b2c_ddot.T, b3c.T) + 2*np.cross(b2c_dot.T, b3c_dot.T)+np.cross(b2c.T, b3c_ddot.T))

        Rc_ddot = np.column_stack((b1c_ddot, b2c_ddot, b3c_ddot)) # desired rotation matrix derivative
        
        #Omega_c_dot_hat = np.dot(Rc.T,Rc_ddot) + np.dot(Omega_c_hat.T,Omega_c_hat) 
        Omega_c_dot_hat = np.dot(Rc.T,Rc_ddot) - np.linalg.matrix_power(Omega_c_hat,2)
        #Omega_c_dot_hat = ar([[0,0,0], [0,0,0], [0, 0, 0]])
        Omega_c_dot = ar([[-Omega_c_dot_hat[1][2]], [Omega_c_dot_hat[0][2]], [-Omega_c_dot_hat[0][1]]])
        
        eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix	
        eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity
        
        #Z = np.dot(np.dot(Omega_hat.dot(R.T),Rc),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 
        Z = np.dot(np.dot(Omega_hat,np.dot(R.T,Rc)),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 

        self.f = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))
        self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))# - np.dot(self.J,Z)

        Msg = control_inputs()
        Msg.header.stamp = rospy.Time.now()
        Msg.Total_thrust = self.f[0][0]
        Msg.Moment_x = self.M[0][0]
        Msg.Moment_y = self.M[1][0] - 0.145  # moment corresponding to offset vi_sensor mass in y direction, slightly different them 0.13 
        Msg.Moment_z = self.M[2][0]
        
        """
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
        """
        self.pub.publish(Msg)
        self.counter+=1

if __name__ == '__main__':
    name = 'firefly'
    rospy.init_node('velocity_controller', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    p = parameters(name)
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]

    try: 
        while not rospy.is_shutdown(): 
            c = velocity_controller(name, parameter_list)
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass

