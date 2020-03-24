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

class vicon_stateestimator_comparison(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
	self.counter = 0
	self.path = '/home/pelican/data_collection/'
	open(self.path + "msckf_estimator.txt", "w").close()
	open(self.path + "vicon.txt", "w").close()
	open(self.path + "vicon_imu_position_comparison.txt", "w").close()
	open(self.path + "vicon_imu_velocity_comparison.txt", "w").close()
	open(self.path + "vicon_imu_rotation_comparison.txt", "w").close()
	
	self.dt = 1.0/25

        self.uav = name_of_uav	
	# The next set of transformations depends on mounting the sensor on the quadrotor. These are approx values
	# but found to be working good with little tweaking. Not the robust way but it works.
	# "self.RMtoG" is the rotation from Map to global frame(in which IMU initializes) of IMU. It remains constant assuming 
	# IMU always initializes same (Patrick said yaw may not be same always but i found very little deviation). 
	# There are minor offsets, I might have missed something, verify it.
	# Map could be any frame and for now its selected as the point on ground from where the quadrotor starts.
	# "self.pGinM" is the position of IMU's global frame in the Map frame. 
	# "self.pCoGinI" is the position of COG(Vicon) in the IMU frame (not the global in which IMU initializes but 
	# actual frame in flight.
	# "self.RItoCoG" is the rotation from IMU (not global) frame to COG. It depends on the angle of the sensor. 
	# I used aprox rpy angles and distances to construct these matrices. Unlucky human being who uses this script 
	# should be able to construct these matrices the way he/she pleases for any configuration. 
	# for velocities the transformation follows the same naming convention   

	self.RMtoG = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # remains same irrespective of orientation of t265
	#self.RMtoG = np.array([[0, -1, 0], [0.9994, 0, -0.0349], [0.0349, 0, 0.9994]]) # a check with -2 deg about y
	self.vGinM = np.array([[0], [0], [0]]) # velocity of G in M remains 0 always

        #self.pGinM = np.array([[0.12], [0], [0.12]])# 0 degree, forward facing
        #self.pCoGinI = np.array([[0.0], [0], [-0.12]]) # 0 degree orientation
	#self.RItoCoG = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) # 0 degree orientation	

        self.pGinM = np.array([[0.12], [0.03], [0.1]])# 10 deg downward
        self.pCoGinI = np.array([[-0.03], [0.0], [-0.12]]) # 10 deg downward
	self.RItoCoG = np.array([[0, 1, 0], [0.1736482, 0, 0.9848077], [0.9848077, 0 , -0.1736482]]) # 10 deg downward
	#self.RItoCoG = np.array([[0, 0.9994, 0.0349], [0.1736482, -0.0349, 0.9842], [0.9848077,  0.0060602, -0.1735424]]) # test with 2 deg in y
	
	#self.pGinM = np.array([[0.13], [0.03], [0.1]])# 45 deg downward
	#self.pCoGinI = np.array([[-0.03], [-0.08], [-0.10]]) # 45 deg downward (these values may be wrong)
	#self.RItoCoG = np.array([[0, -0.8191521,  0.5735765], [-1, 0, 0], [0, -0.5735765, -0.8191521]]) # 45 deg downward

        try:
	    vic = rospy.Subscriber('/udrone1/odom', Odometry, self.viconcallback, queue_size = 10, tcp_nodelay = True)
            data = rospy.Subscriber('/udrone1/modifiedodom', Odometry, self.imucallback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the odometry topic')


    def viconcallback(self, vic):

        t = vic.header.stamp
        #msg.header.stamp = rospy.Time.now()
	self.xvicon = np.array([vic.pose.pose.position.x, vic.pose.pose.position.y, vic.pose.pose.position.z])
	self.qvicon = Quaternion(vic.pose.pose.orientation.w, vic.pose.pose.orientation.x, vic.pose.pose.orientation.y, vic.pose.pose.orientation.z)
	self.vvicon = np.array([vic.twist.twist.linear.x, vic.twist.twist.linear.y, vic.twist.twist.linear.z])		
	# for now I am not comparing angular velocities as Patrick state estimator doesnt publish them and I dont need them 
	# Remember that vicon package from Upenn gives special angular velocities so multiply it with rotation matrix
	#w = np.array([vic.twist.twist.angular.x, vic.twist.twist.angular.y, vic.twist.twist.angular.z])
	R = self.qvicon.rotation_matrix
	#f0 = open(self.path + 'vicon_R.txt', 'a')
	#f0.write("%s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]))
	#self.wvicon = np.dot(R.T, w[np.newaxis])
	f1 = open(self.path + 'vicon.txt', 'a')
	f1.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (t, self.xvicon[0], self.xvicon[1], self.xvicon[2], self.qvicon[1], self.qvicon[2], self.qvicon[3], self.qvicon[0], self.vvicon[0], self.vvicon[1], self.vvicon[2]))


    def rotmat2quat(self, RR): 
        """function to convert a given rotation matrix to a quaternion """ 
        trace = np.trace(RR)
        if trace > 0:
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
	# now we will transform the poses and linear velocities from the state estimator to CoG of the vehicle 
        t = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
	x = data.pose.pose.position.x; y = data.pose.pose.position.y; z = data.pose.pose.position.z # position
	vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z # velocity
	qx = data.pose.pose.orientation.x; qy = data.pose.pose.orientation.y; qz = data.pose.pose.orientation.z; qw = data.pose.pose.orientation.w # orientation

	pCoginM = np.array([[x], [y], [z]])
	vCoginM = np.array([[vx], [vy], [vz]])
        q_cg  = Quaternion(qw, qx, qy, qz)

	f2 = open(self.path + 'msckf_estimator.txt', 'a')
	f2.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (t, pCoginM[0][0], pCoginM[1][0], pCoginM[2][0], q_cg[1], q_cg[2], q_cg[3], q_cg[0], vCoginM[0][0], vCoginM[1][0], vCoginM[2][0]))

	#f3 = open(self.path + 'vicon_imu_position_comparison.txt', 'a')
	#f3.write("%s, %s, %s, %s, %s, %s\n" % (self.xvicon[0], pCoginM[0][0], self.xvicon[1], pCoginM[1][0], self.xvicon[2], pCoginM[2][0]))

	#f4 = open(self.path + 'vicon_imu_velocity_comparison.txt', 'a')
	#f4.write("%s, %s, %s, %s, %s, %s\n" % (self.vvicon[0], vCoginM[0][0], self.vvicon[1], vCoginM[1][0], self.vvicon[2], vCoginM[2][0]))

	#f5 = open(self.path + 'vicon_imu_rotation_comparison.txt', 'a')
	#f5.write("%s, %s, %s, %s, %s, %s, %s, %s\n" % (self.qvicon[1], q_cg[1], self.qvicon[2], q_cg[2], self.qvicon[3], q_cg[3], self.qvicon[0], q_cg[0]))

if __name__ == '__main__':
    name = 'diy2'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('vicon_stateestimator_comparison', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = vicon_stateestimator_comparison(name)
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

