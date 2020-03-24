#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
script transform the state estimates from Patrick's VIO code to the COG frame of the DIY2 vehicle 
and publsihes the correct odometry values at "/diy2/odom" topic.
It also published at "/mavros/vision_pose/pose" topic to unlock GPS lock of pixhawk
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
import time 
import scipy
from scipy import special
from pyquaternion import Quaternion
import tf2_ros


class vio_to_cog(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name, number, freq):
	self.counter = 0
        #self.uav = name_of_uav	
	no = str(number)
	self.dt = 1.0/float(freq)
        self.vlist = []
        self.nsamples_smoothing = 5

	self.RMtoG = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # remains same irrespective of orientation of t265
	self.vGinM = np.array([[0], [0], [0]]) # velocity of G in M remains 0 always

        self.pGinM = np.array([[0.12], [0.03], [0.1]])# 10 deg downward
        self.pCoGinI = np.array([[-0.03], [-0.0], [-0.12]]) # 10 deg downward
	#self.RItoCoG = np.array([[0, 1, 0], [0.1736482, 0, 0.9848077], [0.9848077, 0 , -0.1736482]]) # 10 deg downward
        #self.RItoCoG = np.array([[0, 1, 0], [0.199367, 0, 0.9799], [0.9799, 0 , -0.199367]]) # 11.5 deg downward
        self.RItoCoG = np.array([[0, 1, 0], [0.23344, 0, 0.97237], [0.97237, 0 , -0.23344]]) # 13 deg downward
	#self.RItoCoG = np.array([[0, 1, 0], [0.1305262, 0, 0.9914449], [0.9914449, 0 , -0.1305262]]) # 7.5 deg downward


        self.pub1 = rospy.Publisher('/'+name+no+'/modifiedodom', Odometry, queue_size = 1, tcp_nodelay = True) 
	self.pub2 = rospy.Publisher('/'+name+no+'/mavros/vision_pose/pose', PoseStamped, queue_size = 1, tcp_nodelay = True)
        try:
	    #data = rospy.Subscriber('/'+name+no+'/ov_msckf/odomimu', Odometry, self.odomcallback, tcp_nodelay = True)
	    data = rospy.Subscriber('/'+name+no+'/fastodom', Odometry, self.odomcallback, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the odometry topic of state estimator')


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
    
    def odomcallback(self, data):
	# now we will transform the poses and linear velocities from the state estimator to CoG of the vehicle 
	msg1 = Odometry()
        msg1.header.stamp = data.header.stamp
        msg1.header.frame_id = 'map'
        msg1.child_frame_id = 'mav_cog'
	x = data.pose.pose.position.x; y = data.pose.pose.position.y; z = data.pose.pose.position.z # position
	vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z # velocity
	qx = data.pose.pose.orientation.x; qy = data.pose.pose.orientation.y; qz = data.pose.pose.orientation.z; qw = data.pose.pose.orientation.w # orientation

	pIinG = np.array([[x], [y], [z]])
        q  = Quaternion(qw, qx, qy, qz)
	RGtoI = q.rotation_matrix

	# differentiate rotation matrix numerically for now, ask Patrick to publish angular velocities to get 
	# rid of this differentiation as it depends on dt (freq of publishing) which may not be constant
	# or take the freq as input to the script instead of hardcoding
	if self.counter == 0: 
	    self.RGtoI_dot = np.zeros((3,3)); self.RGtoI_previous = RGtoI
	else: 
	    self.RGtoI_dot = (RGtoI - self.RGtoI_previous)/self.dt; self.RGtoI_previous = RGtoI

	vIinG = np.array([[vx], [vy], [vz]])
	

	RMtoCoG = np.dot(self.RMtoG, np.dot(RGtoI, self.RItoCoG))

	q_cg  = self.rotmat2quat(RMtoCoG)

	pCoGinM = self.pGinM + np.dot(self.RMtoG, pIinG + np.dot(RGtoI, self.pCoGinI))

	vCoGinM = self.vGinM + np.dot(self.RMtoG, vIinG + np.dot(self.RGtoI_dot, self.pCoGinI))
        
        #-----------take an average of few samples for smoothing------------------------
        self.vlist.append(vCoGinM)
        vCoGinM = np.mean(self.vlist, axis = 0)
        if len(self.vlist) >= self.nsamples_smoothing:
            self.vlist.pop(0)
        #-----------take an average of few samples for smoothing------------------------

	msg1.pose.pose.position.x = pCoGinM[0][0]; msg1.pose.pose.position.y = pCoGinM[1][0]
	msg1.pose.pose.position.z = pCoGinM[2][0]
	
	msg1.pose.pose.orientation.x = q_cg[1]; msg1.pose.pose.orientation.y = q_cg[2]
	msg1.pose.pose.orientation.z = q_cg[3]; msg1.pose.pose.orientation.w = q_cg[0]

	msg1.twist.twist.linear.x = vCoGinM[0][0]; msg1.twist.twist.linear.y = vCoGinM[1][0]
	msg1.twist.twist.linear.z = vCoGinM[2][0]

	self.pub1.publish(msg1)

	#----publish the tf between map and odometry frame----------------------------
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	t.header.stamp = data.header.stamp
	t.header.frame_id = "map"
	t.child_frame_id = "mav_cog"
	t.transform.translation.x = pCoGinM[0][0]; t.transform.translation.y = pCoGinM[1][0]
	t.transform.translation.z = pCoGinM[2][0]

	
	t.transform.rotation.x = q_cg[1]; t.transform.rotation.y = q_cg[2]
	t.transform.rotation.z = q_cg[3]; t.transform.rotation.w = q_cg[0]
	br.sendTransform(t)
	#------------this is to get pointclouds and octomap in the map frame------------------	

	#------publish to /mavros/vision_pose/pose ----------------------------------------
	msg2 = PoseStamped()
	msg2.header.stamp = data.header.stamp
	msg2.pose.position.x = pCoGinM[0][0]; msg2.pose.position.y = pCoGinM[1][0]
	msg2.pose.position.z = pCoGinM[2][0]
        #q_cg = Quaternion(0.707, 0, 0, 0.707)*q_cg
	msg2.pose.orientation.x = q_cg[1]; msg2.pose.orientation.y = q_cg[2]
	msg2.pose.orientation.z = q_cg[3]; msg2.pose.orientation.w = q_cg[0]
	self.pub2.publish(msg2)
	self.counter += 1

if __name__ == '__main__':
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    freq = rospy.get_param('freq')
    rospy.init_node('vio_to_cog', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = vio_to_cog(name, number, freq)
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

