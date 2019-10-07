#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 23 10:46:20 2018

@author: indrajeet
This script is used to start stop the motors of DIY quadrotor using px4. It 
will eventually be modified (or used in some form) to include full controller. 
"""

import numpy as np 
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
import message_filters
from isy_geometric_controller.msg import Desired_Trajectory
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
from mavros_msgs.msg import ActuatorControl, Thrust, AttitudeTarget
import rowan 
from sensor_msgs.msg import Imu

class px4_offboard_control(object):
    def __init__(self, name_of_uav):
        """initializes the variables"""
        self.counter = 0
        self.g = 9.86; self.m = 1.61; self.dt = 1.0/100
        self.kx = np.array([[5], [5], [5]])
        self.kv = np.array([[2.2], [2.2], [2.2]])
        self.e = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.max_possible_thrust = 36.0 # determined by testing
        self.rate_gain = 0.1
        self.factor = 1.0/self.rate_gain
        self.angular_velocity = ar([[0], [0],[0]]) 
        self.linear_acceleration = ar([[0], [0],[0]])
        
        self.pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 10)

        try: 
            self.arming_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self.arming_service.call(True)
            #self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            #self.set_mode_service.call("OFFBOARD")
            
            rospy.Subscriber('/'+name_of_uav+'/odom', Odometry, self.command_callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imu_callback, queue_size = 1, tcp_nodelay = True)
            rospy.Subscriber('/desired_trajectory', Desired_Trajectory, self.traj_callback, queue_size = 1, tcp_nodelay = True)
            
            
            #odom = message_filters.Subscriber('/'+name_of_uav+'/odom', Odometry, tcp_nodelay = True)
            #traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory, tcp_nodelay = True)
            #ts = message_filters.TimeSynchronizer([odom, traj], 100)
            #ts.registerCallback(self.send_commands)
            rospy.spin()
        except: 
            print "Probem subscribing to mocap topic"
    def traj_callback(self, traj): 
        """takes the trajectory"""
        self.pdes = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
        self.vdes = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        self.ades = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        self.ddes = ar([[traj.desired_direction.x], [traj.desired_direction.y], [0]]) # ignore third component
        
    def imu_callback(self, data): 
        """takes the data from imu"""
        self.angular_velocity = ar([[data.angular_velocity.x], [data.angular_velocity.y],[data.angular_velocity.z]]) 
        self.linear_acceleration = ar([[data.linear_acceleration.x], [data.linear_acceleration.y],[data.linear_acceleration.z]])
        
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
 
        return Quaternion(qw, qx, qy, qz)
        
    def command_callback(self, odom):
        """send command to px4"""
        
        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])

        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = self.angular_velocity
        #Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        #Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities


        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = self.pdes; Vd = self.vdes; ad = self.ades; b1d = self.ddes        

        
        ex = ar([[0],[0],[0]]) if traj.controller == 1 else X-Xd
        ev = V-Vd

        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev)) + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector

        self.f = self.m* np.dot(_b3c.T, tp(R[:,2][np.newaxis]))/self.max_possible_thrust # normalized thrust 
        
        yaw = np.arctan2(b2c[1][0], b2c[0][0])
        yaw  = yaw + 2 * np.pi if yaw < 0 else yaw # vary yaw between 0 and 360 degree
        
        # find rate of change of  yaw and force vector 
        if self.counter != 0:
            yaw_dot = (yaw-self.yaw_)/self.dt; self.yaw_ = yaw
            _b3c_dot = (_b3c-self._b3c_previous)/self.dt; self._b3c_previous = _b3c
        else: 
            yaw_dot = 0; self.yaw_ = yaw
            _b3c_dot = ar([[0], [0], [0]]); self._b3c_previous = _b3c
            
        b1d = ar([[np.cos(yaw)], [np.sin(yaw)], [0]])  
        b1d_dot = ar([[-np.sin(yaw)*yaw_dot], [np.cos(yaw)*yaw_dot], [0]])
        
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix
        
        # take the derivatives
        middle1 = _b3c_dot/np.linalg.norm(_b3c)
        last_two = np.cross(middle1.T, b3c.T)
        b3c_dot = np.cross(b3c.T, last_two)
        
        middle2 = (np.cross(b3c_dot, b1d.T) + np.cross(b3c.T, b1d_dot.T))/np.linalg.norm(np.cross(b3c.T, b1d.T))
        last_two = np.cross(middle2, b2c.T)
        b2c_dot = np.cross(b2c.T, last_two)
        
        b1c_dot = np.cross(b2c_dot, b3c.T) + np.cross(b2c.T, b3c_dot)
        Rc_dot = np.column_stack((b1c_dot.T, b2c_dot.T, b3c_dot.T)) # desired rotation matrix
        
        Omega_des = np.dot(Rc.T, Rc_dot)
        
        
        """
        qc = self.rotmat2quat(Rc)
        omega_des = q.inverse*qc
        phi = np.arctan2(b2c[1][0], b2c[0][0])
        if phi < 0: 
            phi = phi + 2 * np.pi
        #error  = 0.5 * np.trace(self.e-np.dot(Rc.T, R))

        if self.counter != 0:
            phi_dot = (phi-self.phi_)/self.dt; self.phi_ = phi
        else: 
            phi_dot = 0; self.phi_ = phi

        Omega_c = ar([[0], [0], [phi_dot]])
        """
        #eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix
        #
        #eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        #eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity

        
        
        #self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))
        msg = AttitudeTarget()
        msg.header.stamp = odom.header.stamp
        msg.type_mask = 128 


        msg.body_rate.x = Omega_des[1][2]
        msg.body_rate.y = -Omega_des[0][2]
        msg.body_rate.z = -Omega_des[0][1]
        
        #msg.body_rate.x = self.factor*np.sign(omega_des[0])*omega_des[1]
        #msg.body_rate.y = -self.factor*np.sign(omega_des[0])*omega_des[2]
        #msg.body_rate.z = -self.factor*np.sign(omega_des[0])*omega_des[3]

        msg.thrust = min(1.0, self.f[0][0])
        print 'thrust', min(1.0, self.f[0][0])
        print 'omega_des',msg.body_rate.x, msg.body_rate.y, msg.body_rate.z
        print 'zheight', traj.desired_position.z
        
	#f0 = open('commands.txt', 'a')
	#f0.write("%s, %s, %s, %s\n" % (msg.thrust, msg.body_rate.x, msg.body_rate.y, msg.body_rate.z)) 
        self.counter = self.counter+1
        self.pub.publish(msg)

        

if __name__ == '__main__':
    name = 'hummingbird'
    rospy.init_node('sendcommandtopx4', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    c = px4_offboard_control(name)
    try: 
        while not rospy.is_shutdown():             
            #rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
