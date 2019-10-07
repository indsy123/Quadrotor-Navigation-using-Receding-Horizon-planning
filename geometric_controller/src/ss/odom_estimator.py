#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 17:28:19 2017
This initial script generates a dummy trajectory. This should be replaced 
eventually with what you want the trajectory to be or your own method to 
generate the trajectory. 
Basically I made my own message file called "Desired_trajectory" 
that is a message type have position, velocity, acceleration and direction. 
Velocity and acceleration need not be here as Lee's paper says the 
trajectory is an (x,y,z) position of CoG and a direction. 
The current trajectory is the one used in example (1) in the paper  
"Geomentric tracking control of a quadrotor in SE(3)" by T Lee. 
You can change it the way you want. just get a curve [x(t),y(t),z(t)] and a 
direction [cos(pi*t), sin(pi*t),0] or as you fancy. Differentiate the x, y and
z to get velocities and accelerations. 
While it is possible to get rid of velocities and accelerations here and 
calculate them in the controller script,I found it was not resulting in much 
saving in terms of time.
It will also be needed to change queue_size and publishing frequency in 
"r = rospy.Rate(n). With this function my laptop can generate at the most 
155 hz.
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
import rospy
#from isy_geometric_controller.msg import Desired_Trajectory
#from isy_geometric_controller.msg import modifiedodometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time 
import scipy
from scipy import special
from pyquaternion import Quaternion

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
	self.counter = 0
	open("new_msckf_estimator.txt", "w").close()
	open("vicon_imu_comparison.txt", "w").close()

        self.uav = name_of_uav
	self.pVinI = np.array([[0.0], [0], [-0.12]])
	self.pGinM = np.array([[0.12], [0], [0.12]])
	#self.RMtoG = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
	self.RMtoG = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
	self.RItoV = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])	


        try:
	    vic = rospy.Subscriber('/diy2/odom', Odometry, self.viconcallback, queue_size = 10, tcp_nodelay = True)
            data = rospy.Subscriber('/ov_msckf/poseimu', PoseWithCovarianceStamped, self.imucallback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    def viconcallback(self, vic):

        #t = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
	self.vx = vic.pose.pose.position.x; self.vy = vic.pose.pose.position.y; self.vz = vic.pose.pose.position.z
	self.vqx = vic.pose.pose.orientation.x; self.vqy = vic.pose.pose.orientation.y; self.vqz = vic.pose.pose.orientation.z; self.vqw = vic.pose.pose.orientation.w;
	#vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z
	#wx = data.twist.twist.angular.x; wy = data.twist.twist.angular.y; wz = data.twist.twist.angular.z


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
    
    def imucallback(self, data):

        t = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
	x = data.pose.pose.position.x; y = data.pose.pose.position.y; z = data.pose.pose.position.z
	qx = data.pose.pose.orientation.x; qy = data.pose.pose.orientation.y; qz = data.pose.pose.orientation.z; qw = data.pose.pose.orientation.w
	pIinG = np.array([[x], [y], [z]])
        q  = Quaternion(qw, qx, qy, qz)
	RGtoI = q.rotation_matrix
	
	RMtoV = np.dot(self.RMtoG, np.dot(self.RItoV, RGtoI))
	q_cg  = self.rotmat2quat(RMtoV)

	pVinM = self.pGinM + np.dot(self.RMtoG, pIinG + np.dot(RGtoI, self.pVinI))

	f1 = open('new_msckf_estimator.txt', 'a')
	#f1.write("%s, %s, %s, %s, %s, %s, %s, %s\n" % (t, x, y, z, qx, qy, qz, qw))
	f1.write("%s, %s, %s, %s, %s, %s, %s\n" % (pVinM[0][0], pVinM[1][0], pVinM[2][0], q_cg[1], q_cg[2], q_cg[3], q_cg[0]))

	f2 = open('vicon_imu_comparison.txt', 'a')
	f2.write("%s, %s, %s, %s, %s, %s\n" % (pVinM[0][0], self.vx, pVinM[1][0], self.vy, pVinM[2][0], self.vz))



if __name__ == '__main__':
    name = 'diy'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('estimator_odom', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = trajectory(name)
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

