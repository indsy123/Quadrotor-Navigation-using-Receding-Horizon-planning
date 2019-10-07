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
#from scipy.interpolate import splrep, splev

class position_controller(object): 
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
        #self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size = 1)
        self.pub = rospy.Publisher('/control_inputs', control_inputs, queue_size = 100)
        #self.ad_series = []
        #self.ad_dot_series = []
        #self.acc_series = []
        #self.timeseries = []
        try:
            #odom = message_filters.Subscriber('/'  +self.uav + '/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
            #traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory, tcp_nodelay = True)
            #accel = message_filters.Subscriber('/'+ self.uav + '/ground_truth/imu', Imu, tcp_nodelay = True)
            #ts = message_filters.TimeSynchronizer([odom, accel, traj], 100)
            odom1 = message_filters.Subscriber('/'+self.uav+'/vi_sensor/ground_truth/odometry', Odometry, tcp_nodelay = True)
            odom2 = message_filters.Subscriber('/'+self.uav+'/ground_truth/odometry', Odometry, tcp_nodelay = True)
            #traj = message_filters.Subscriber('/NavigationFunctionTrajectory', velocities_from_navigation_function, tcp_nodelay = True)
            traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory, tcp_nodelay = True)            
            accel = message_filters.Subscriber('/'+self.uav+'/ground_truth/imu', Imu, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([odom1, odom2, accel, traj], 100)
            ts.registerCallback(self.callback)
            rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
   
    def callback(self, odom2, odom, accel, traj): 



        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        
        
        # next two matrices are position and orientation offset of the vi sensor imu with the center of gravity
        #vi_pos_off_hat = ar([[0.0, 0.03, 0.0], [-0.03, 0.0, -0.1], [0.0, 0.1, 0.0]]) 
        #vi_rot_off = ar([[0.9950042, 0.0, 0.0998334], [0.0, 1.0, 0.0], [-0.0998334, 0.0, 0.9950042]])
        
        #intermediate = np.dot(-vi_pos_off_hat, vi_rot_off.T)
        #V11 = _V + np.dot(vi_pos_off_hat, Omega)  
        
        #Omega = np.dot(vi_rot_off.T, Omega)
        Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0],0,-Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])
        
        #VV =  ar([[odom2.twist.twist.linear.x], [odom2.twist.twist.linear.y], [odom2.twist.twist.linear.z]])       
        
        _acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z - 9.8]])
        #print '1', R1
        #R = np.dot(vi_rot_off.T, R1)
        #print '2', R
        #ff = open('rotation.txt', 'a')
        #ff.write("%s, %s\n" %(R1, R)) 
        V1 = np.dot(R, _V)
        acc = np.dot(R, _acc)
        #f6 = open('velocity_imu_cog.txt', 'a')
        #f6.write("%s, %s, %s, %s, %s, %s\n" % (V11[0][0], V11[1][0], V11[2][0], V1[0][0], V1[1][0], V1[2][0]))
        
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
 
        Vd = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        ad = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        ad_dot = ar([[0], [0], [0]])
        #ad_dot = ar([[traj.desired_jerk.x], [traj.desired_jerk.y], [traj.desired_jerk.z]])

        b1d = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])
        #b1d = ar([[1], [1], [0]])
        b1d_dot = ar([[0], [0], [0]])
        b1d_ddot = ar([[0], [0], [0]])

        #current_time = time.time()
        #t = current_time - self.time
        #self.time_points.append(t)
        """
        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]]);
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        V = np.dot(R, _V)
        acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
        #acc = np.dot(R, _acc)
        #print acc
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        #Omega = np.dot(R, _Omega)

        Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0],0,-Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])
        
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
 
        Vd = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        ad = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        b1d = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])

        b1d_dot = ar([[traj.desired_direction_dot.x], [traj.desired_direction_dot.y], [traj.desired_direction_dot.z]])
        b1d_ddot = ar([[traj.desired_direction_ddot.x], [traj.desired_direction_ddot.y], [traj.desired_direction_ddot.z]])
        
        f0= open('trajectory.txt', 'a')
        f0.write("%s, %s, %s, %s, %s, %s\n" % (X[0][0], X[1][0], X[2][0], Xd[0][0], Xd[1][0], Xd[2][0]))
        """
        #correction = np.array([[0],[0],[0.1]])
        ex = X-Xd#-correction 
        ev = V1-Vd
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction 
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix

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
        """
            
        
        if self.counter != 0:
            self.current_time = time.time()
            delt = self.current_time-self.previous_time
            ad_dot = (ad-self._ad)/delt; ad_ddot = (ad_dot-self._ad_dot)/delt   
            V_ddot = (acc-self.acc_)/delt
            self._ad = ad; self.acc_ = acc; self._ad_dot = ad_dot
            self.previous_time = time.time()
        else: 
            ad_dot = tp(np.zeros((1,3))); self._ad = ad
            V_ddot = tp(np.zeros((1,3))); self.acc_ = acc; ad_ddot = tp(np.zeros((1,3))); self._ad_dot = ad_dot
            self.previous_time = time.time()  


        f0 = open('velocity.txt', 'a')
        f0.write("%s, %s, %s, %s, %s, %s\n" % (V1[0][0], V1[1][0], V1[2][0], Vd[0][0], Vd[1][0], Vd[2][0]))
        f1 = open('accel.txt', 'a')
        f1.write("%s, %s, %s, %s, %s, %s\n" % (acc[0][0], acc[1][0], acc[2][0], ad[0][0], ad[1][0], ad[2][0]))
        f2 = open('position.txt', 'a')
        f2.write("%s, %s, %s, %s, %s, %s\n" % (X[0][0], X[1][0], X[2][0], Xd[0][0], Xd[1][0], Xd[2][0]))


        """
        
        if self.counter == 0: 
            ad_dot = tp(np.zeros((1,3))); ad_ddot = tp(np.zeros((1,3))); V_ddot = tp(np.zeros((1,3)))
            self.ad_series.append(ad); self.ad_dot_series.append(ad_dot)
            self.acc_series.append(acc) ; self.timeseries.append(time.time())

        elif self.counter == 1:
            delt = time.time()-self.timeseries[-1]
            ad_dot = (ad-self.ad_series[-1])/delt; ad_ddot = (ad_dot-self.ad_dot_series[-1])/delt   
            V_ddot = (acc-self.acc_series[-1])/delt
            self.ad_series.append(ad); self.ad_dot_series.append(ad_dot)
            self.acc_series.append(acc) ; self.timeseries.append(time.time())
             
        else:
            delt = (time.time()-self.timeseries[-2])/2
            
            ad_dot = (3*ad-4*self.ad_series[-1]+self.ad_series[-2])/(2*delt)
            ad_ddot = (3*ad_dot-4*self.ad_dot_series[-1]+self.ad_dot_series[-2])/(2*delt)
            V_ddot = (3*acc-4*self.acc_series[-1]+self.acc_series[-2])/(2*delt)
            self.ad_series.append(ad); self.ad_dot_series.append(ad_dot)
            self.acc_series.append(acc) ; self.timeseries.append(time.time())
            self.ad_series.pop(0); self.ad_dot_series.pop(0); self.acc_series.pop(0); self.timeseries.pop(0)
        """
            
        A = (mult(self.kx, ex) + mult(self.kv, ev))/self.m - self.g*self.e[:,2][np.newaxis].T - ad
        ex_dot =  ev ; ev_dot = acc-ad
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

        
        ex_ddot = ev_dot; ev_ddot = V_ddot-ad_dot
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
        self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega)))) #- np.dot(self.J,Z)
        #print self.M
        Msg = control_inputs()
        Msg.header.stamp = rospy.Time.now()
        Msg.Total_thrust = self.f[0][0]
        Msg.Moment_x = self.M[0][0]
        Msg.Moment_y = self.M[1][0]
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
    rospy.init_node('position_controller', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    p = parameters(name)
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]

    try: 
        while not rospy.is_shutdown(): 
            c = position_controller(name, parameter_list)
            rospy.spin()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass


