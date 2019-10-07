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
        self.norm_thrust_constant = 36.0
        self.rate_gain = 0.1
        self.factor = 2.0/self.rate_gain
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
        self.ddes = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])
        #self.ddes = ar([[traj.desired_direction.x], [traj.desired_direction.y], [0]]) # ignore third component
        
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

        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector

        #yaw = np.arccos(np.dot(e[:,0][np.newaxis], b1d)/np.linalg.norm(b1d))
        #b1d = ar([[np.cos(yaw[0][0])], [np.sin(yaw[0][0])], [0]])
        
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix
        #qc = self.rotmat2quat(Rc)
        #qc = rowan.to_matrix(Rc)
        qc = self.rotmat2quat(Rc)
        
        #print 'qc', qc
        
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

        #eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix
        #
        #eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        #eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity

        self.f = np.dot(_b3c.T, tp(R[:,2][np.newaxis]))/self.norm_thrust_constant # normalized thrust 
        
        #self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))
        msg = AttitudeTarget()
        msg.header.stamp = odom.header.stamp
        msg.type_mask = 128 
        
        msg.body_rate.x = self.factor*np.sign(omega_des[0])*omega_des[1]
        msg.body_rate.y = -self.factor*np.sign(omega_des[0])*omega_des[2]
        msg.body_rate.z = -self.factor*np.sign(omega_des[0])*omega_des[3]

        msg.thrust = min(1.0, self.f[0][0])
        print 'thrust', min(1.0, self.f[0][0])
        print 'omega_des',msg.body_rate.x, msg.body_rate.y, msg.body_rate.z
        print 'zheight', traj.desired_position.z
        
	#f0 = open('commands.txt', 'a')
	#f0.write("%s, %s, %s, %s\n" % (msg.thrust, msg.body_rate.x, msg.body_rate.y, msg.body_rate.z)) 
        self.counter = self.counter+1
        self.pub.publish(msg)
        """
        Msg = control_inputs()
        #Msg.header.stamp = rospy.Time.now()
        Msg.header.stamp = odom.header.stamp
        Msg.Total_thrust = self.f[0][0]
        Msg.Moment_x = self.M[0][0]
        Msg.Moment_y = self.M[1][0] #- 0.145  # moment corresponding to offset vi_sensor mass in y direction, slightly different them 0.13 
        Msg.Moment_z = self.M[2][0]
        

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

        Msg = PoseStamped()
        Msg.header.stamp = data.header.stamp
        Msg.pose.position.x = 0; Msg.pose.position.y = 0; Msg.pose.position.z = 0.5;
        Msg.pose.orientation.x = 0; Msg.pose.orientation.y = 0; Msg.pose.orientation.z = 0; Msg.pose.orientation.w = 1
        """
      
        

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
