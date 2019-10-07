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
from geometric_controller.msg import Desired_Trajectory, control_inputs, modifiedodometry
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
from numpy import sin as sin
from numpy import cos as cos
from numpy import pi as pi 
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators
from geometric_controller.msg import PolynomialTrajectory, velocities_from_navigation_function
#from scipy.interpolate import splrep, splev

class velocity_controller(object): 
    def __init__(self, name_of_uav, number, parameters, freq): 
        self.uav = name_of_uav
        self.no = str(number)
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
	self.dt = 1.0/freq # time step based on 150Hz frequency
	self.ei = 0
	#self.ki = np.array([[1], [1], [1]]) 
	#self.ki_att = np.array([[3], [3], [3]])
        #self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size = 1)
        self.pub = rospy.Publisher('/firefly'+self.no+'/command/motor_speed', Actuators, queue_size = 1, tcp_nodelay = True)
        try:
            odom = message_filters.Subscriber('/firefly'+self.no+'/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
	    #odom = message_filters.Subscriber('/'+self.uav+'/odom', Odometry, tcp_nodelay = True)
            traj = message_filters.Subscriber('/firefly'+self.no+'/NavigationFunctionTrajectory', velocities_from_navigation_function, tcp_nodelay = True)
            #odom = message_filters.Subscriber('/omnipuck/odom', Odometry, tcp_nodelay = True)            
            #traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory, tcp_nodelay = True) 
            #accel = message_filters.Subscriber('/'+self.uav+'/ground_truth/imu', Imu, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([odom, traj], 10)
            ts.registerCallback(self.callback)
            #rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
   
    def callback(self, odom, traj): 
        
        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        #acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
        #V1 = _V#np.dot(R, _V);
	V1 = np.dot(R, _V);
        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
	#Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities
	"""
        X = ar([[odom.pose.position.x], [odom.pose.position.y], [odom.pose.position.z]])
        q = Quaternion(odom.pose.orientation.w, odom.pose.orientation.x, odom.pose.orientation.y, odom.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.linear.x], [odom.twist.linear.y], [odom.twist.linear.z]])
        #acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
        V1 = _V#np.dot(R, _V)        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.angular.x], [odom.twist.angular.y], [odom.twist.angular.z]])
	Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities
	"""

	"""
        # next two matrices are position and orientation offset of the vi sensor imu with the center of gravity
        vi_pos_off_hat = ar([[0.0, 0.03, 0.0], [-0.03, 0.0, -0.1], [0.0, 0.1, 0.0]]) 
        vi_rot_off = ar([[0.9950042, 0.0, 0.0998334], [0.0, 1.0, 0.0], [-0.0998334, 0.0, 0.9950042]])
        #vi_rot_off = ar([[0.7071, 0.0, 0.7071], [0.0, 1.0, 0.0], [-0.7071, 0.0, 0.7071]])

        intermediate = np.dot(vi_pos_off_hat, vi_rot_off)
        V1 = np.dot(vi_rot_off,_V) + np.dot(intermediate, Omega)         
        Omega = np.dot(vi_rot_off, Omega)
        R = np.dot(R, vi_rot_off.T)
        
        #Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0], 0, -Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])
        V1 = np.dot(R, V1)
        #acc = np.dot(R, acc)
	"""

     
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[traj.pdes.x], [traj.pdes.y], [traj.pdes.z]])
        Vd = ar([[traj.vdes.x], [traj.vdes.y], [traj.vdes.z]])
        ad = ar([[traj.ades.x], [traj.ades.y], [traj.ades.z]])
        #ad_dot = ar([[traj.jdes.x], [traj.jdes.y], [traj.jdes.z]])

        b1d = ar([[traj.ddes.x], [traj.ddes.y], [traj.ddes.z]])
        #b1d_dot = ar([[traj.ddes_dot.x], [traj.ddes_dot.y], [traj.ddes_dot.z]])

        
        if traj.controller == 1: # velocity controller
            ex = ar([[0],[0],[0]])
        else: 
            ex = X-Xd
        
        ev = V1-Vd

        #_b3c = -(mult(self.kx, ex) + mult(self.kv, ev)+ mult(self.ki, ei))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction 
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix

        phi = np.arctan2(b2c[1][0], b2c[0][0])

        if phi < 0: 
            phi = phi + 2 * np.pi

        error  = 0.5 * np.trace(self.e-np.dot(Rc.T, R))

        
        if self.counter != 0:
            phi_dot = (phi-self.phi_)/self.dt; self.phi_ = phi
        else: 

            phi_dot = 0; self.phi_ = phi

        Omega_c = ar([[0], [0], [phi_dot]])
        """
        if self.counter == 0:
            acc = tp(np.zeros((1,3))); self.previous_V = V1
        else: 
            acc = (V1-self.previous_V)/self.dt; self.previous_V = V1
            
        # calculate derivative of rotation and find desired angular velocity        
        A = (mult(self.kx, ex) + mult(self.kv, ev))/self.m - self.g*self.e[:,2][np.newaxis].T - ad
        ex_dot = ev; ev_dot = acc-ad
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
        """
        eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix

        eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity

	#if self.counter==0:
	#    ei_att = 0; self.ei_att = ei_att
	#else: 	
	#    ei_att = (eR+eOmega)*self.dt
	#    ei_att = self.ei_att+ei_att
	#    self.ei_att = ei_att
        
        #Z = np.dot(np.dot(Omega_hat.dot(R.T),Rc),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 
        #Z = np.dot(np.dot(Omega_hat,np.dot(R.T,Rc)),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 

        self.f = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))
	#self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)+mult(self.ki_att, ei_att)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))# - np.dot(self.J,Z)
        self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))# - np.dot(self.J,Z)

        T = tp(ar([[self.M[0][0],self.M[1][0],self.M[2][0],self.f[0][0]]]))
	Msg = Actuators()
	if self.uav == 'hummingbird':
	    c1 = tp(ar([[0, 2/(self.d*self.tau_t), 0, -2/(self.d*self.tau_t)]]))
            c2 = tp(ar([[-2/(self.d*self.tau_t), 0, 2/(self.d*self.tau_t), 0]]))
            c3 = tp(ar([[1/(self.tau_t*self.m), -1/(self.tau_t*self.m), 1/(self.tau_t*self.m), -1/(self.tau_t*self.m)]]))
            c4 = tp(ar([[1/self.tau_t, 1/self.tau_t, 1/self.tau_t, 1/self.tau_t]]))
            C = 0.25*np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2 
            w_square = np.dot(C,T)
            #w_initial = np.array([[self.w0_h**2], [self.w0_h**2], [self.w0_h**2], [self.w0_h**2]])
            w = np.sqrt(np.abs(w_square))
            #w[w>800.0] = 800.0
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]

	elif self.uav== 'pelican': 

	    c1 = tp(ar([[0, 2/(self.d*self.tau_t), 0, -2/(self.d*self.tau_t)]]))
            c2 = tp(ar([[-2/(self.d*self.tau_t), 0, 2/(self.d*self.tau_t), 0]]))
            c3 = tp(ar([[1/(self.tau_t*self.m), -1/(self.tau_t*self.m), 1/(self.tau_t*self.m), -1/(self.tau_t*self.m)]]))
            c4 = tp(ar([[1/self.tau_t, 1/self.tau_t, 1/self.tau_t, 1/self.tau_t]]))
            C = 0.25*np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2 
            w_square = np.dot(C,T)
	    #w_initial = np.array([[self.w0_p**2], [self.w0_p**2], [self.w0_p**2], [self.w0_p**2]])
            #w = np.sqrt(np.abs(w_square+w_initial))
            w = np.sqrt(np.abs(w_square))
            #w[w>800.0] = 800.0
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]

        else: 
            c1 = tp(ar([[sin(pi/6)*self.d*self.tau_t, -cos(pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
            c2 = tp(ar([[sin(pi/2)*self.d*self.tau_t, -cos(pi/2)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
            c3 = tp(ar([[sin(5*pi/6)*self.d*self.tau_t, -cos(5*pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
            c4 = tp(ar([[sin(7*pi/6)*self.d*self.tau_t, -cos(7*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
            c5 = tp(ar([[sin(3*pi/2)*self.d*self.tau_t, -cos(3*pi/2)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
            c6 = tp(ar([[sin(11*pi/6)*self.d*self.tau_t, -cos(11*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
            C = np.column_stack((c1,c2,c3,c4,c5,c6)) # solving linear eq T = Cw^2 to get w^2
            inverted_matrix = np.linalg.pinv(C)#np.dot(C.T, np.linalg.inv(np.dot(C, C.T)))
            w_square = np.dot(inverted_matrix,T)
	    #w_initial = np.array([[self.w0_f**2], [self.w0_f**2], [self.w0_f**2], [self.w0_f**2], [self.w0_f**2], [self.w0_f**2]])
            #w = np.sqrt(np.abs(w_square+w_initial))
            w = np.sqrt(np.abs(w_square))
            #w[w>900.0] = 900.0
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0], w[4][0], w[5][0]]
        
        self.pub.publish(Msg)
        self.counter+=1

if __name__ == '__main__':
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    freq = rospy.get_param('publish_frequency')
    rospy.init_node('PositionVelocityController', anonymous=False)
    r = rospy.Rate(freq)
    p = parameters(name)
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]
    c = velocity_controller(name, number, parameter_list, freq)
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass

