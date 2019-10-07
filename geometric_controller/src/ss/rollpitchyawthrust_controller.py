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
from isy_geometric_controller.msg import Desired_Trajectory, control_inputs, modifiedodometry
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators
from isy_geometric_controller.msg import velocities_from_navigation_function
from scipy.interpolate import splrep, splev
from std_msgs.msg import Float32MultiArray

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
	self.dt = 1.0/150 # time step based on 200Hz frequency 
	self.ei = 0
	self.ki = np.array([[1], [1], [1]]) 
	self.ki_att = np.array([[3], [3], [3]])
        #self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size = 1)
        self.pub = rospy.Publisher('/rollpitchyawthrust', Float32MultiArray, queue_size = 10)
        try:
            #odom = message_filters.Subscriber('/'+self.uav+'/odometry_sensor1/odometry', Odometry, tcp_nodelay = True)
	    odom = message_filters.Subscriber('/'+self.uav+'/modified/odom', modifiedodometry, tcp_nodelay = True)
            #traj = message_filters.Subscriber('/NavigationFunctionTrajectory', velocities_from_navigation_function, tcp_nodelay = True)
            #odom = message_filters.Subscriber('/omnipuck/odom', Odometry, tcp_nodelay = True)            
            traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory, tcp_nodelay = True) 
            #accel = message_filters.Subscriber('/'+self.uav+'/ground_truth/imu', Imu, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([odom, traj], 100)
            ts.registerCallback(self.callback)
            rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
   
    def callback(self, odom, traj): 
        """
        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        #acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
        V1 = _V#np.dot(R, _V);
        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
	Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities
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
        Xd = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
        Vd = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        ad = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        #ad_dot = ar([[traj.desired_jerk.x], [traj.desired_jerk.y], [traj.desired_jerk.z]])

        b1d = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])
        #if traj.controller == 0: 
        #    phi = 0
        #else:
        #    phi = np.arctan2(b1d[1][0], b1d[0][0])
        #if phi < 0: 
        #    phi = phi + 2 * np.pi

        #correction = np.array([[0],[0],[0.1]])
        
        if traj.controller == 1: # velocity controller
            ex = ar([[0],[0],[0]])
        else: 
            ex = X-Xd
        
        ev = V1-Vd
	if self.counter==0:
	    ei = 0; self.ei = ei
	else: 	
	    ei = (ex+ev)*self.dt
	    ei = self.ei+ei
	    self.ei = ei 
        #_b3c = -(mult(self.kx, ex) + mult(self.kv, ev)+ mult(self.ki, ei))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction 
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix


        phi = np.arctan2(b2c[1][0], b2c[0][0])
	#print "phi:1", phi
        if phi < 0: 
            phi = phi + 2 * np.pi
	#print "phi:2", phi

        yaw = phi # yaw angle         
	roll = np.arctan(Rc[1][0], Rc[0][0])*180.0/np.pi
	pitch = np.arctan2(-Rc[2][0]/np.sqrt((Rc[2][1])**2+(Rc[2][2])**2))

        error  = 0.5 * np.trace(self.e-np.dot(Rc.T, R))

	#print error
        
        #current_time = time.time()
        #t = current_time - self.time
        #self.time_points.append(t)
        
        if self.counter != 0:

            phi_dot = (phi-self.phi_)/self.dt; self.phi_ = phi
        else: 
            #ad_dot = tp(np.zeros((1,3))); 
            #self._ad = ad
            phi_dot = 0; self.phi_ = phi
      
        
        Omega_c = ar([[0], [0], [phi_dot]])

        eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix

        eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity

	if self.counter==0:
	    ei_att = 0; self.ei_att = ei_att
	else: 	
	    ei_att = (eR+eOmega)*self.dt
	    ei_att = self.ei_att+ei_att
	    self.ei_att = ei_att
        
        #Z = np.dot(np.dot(Omega_hat.dot(R.T),Rc),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 
        #Z = np.dot(np.dot(Omega_hat,np.dot(R.T,Rc)),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 

        self.f = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))
	#self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)+mult(self.ki_att, ei_att)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))# - np.dot(self.J,Z)
        self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))# - np.dot(self.J,Z)

        Msg = control_inputs()
        #Msg.header.stamp = rospy.Time.now()
	Msg.header.stamp = odom.header.stamp
        Msg.Total_thrust = self.f[0][0]
        Msg.Moment_x = self.M[0][0]
        Msg.Moment_y = self.M[1][0] #- 0.145  # moment corresponding to offset vi_sensor mass in y direction, slightly different them 0.13 
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
    name = 'hummingbird'
    #name = rospy.get_param('~vehicle_name')
    #print '*******************************************88',name
    rospy.init_node(name+'_PositionVelocityController', anonymous=False, log_level=rospy.DEBUG)
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

