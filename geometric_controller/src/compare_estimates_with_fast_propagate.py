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

class compare(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, RN):
        self.RN = RN
 	self.counter = 0
	self.path = '/home/pelican/data_collection/rhp_logs/'
	open(self.path+"est_fp_position_RN{}.txt".format(RN), "w").close()
	open(self.path+"est_fp_velocity_RN{}.txt".format(RN), "w").close()
 	open(self.path+"est_fp_orientation_RN{}.txt".format(RN), "w").close()

        try:
            rospy.Subscriber('/udrone1/ov_msckf/odomimu', Odometry, self.estcallback, tcp_nodelay = True)
            rospy.Subscriber('/udrone1/fastodom', Odometry, self.fastpropcallback, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    def estcallback(self, est):
        self.p_est = np.array([est.pose.pose.position.x, est.pose.pose.position.y, est.pose.pose.position.z])
        self.q_est = np.array([est.pose.pose.orientation.x, est.pose.pose.orientation.y, est.pose.pose.orientation.z, est.pose.pose.orientation.w])
        self.v_est = np.array([est.twist.twist.linear.x, est.twist.twist.linear.y, est.twist.twist.linear.z])



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
    
    def fastpropcallback(self, fp):

        #t = data.header.stamp
        self.p_fp = np.array([fp.pose.pose.position.x, fp.pose.pose.position.y, fp.pose.pose.position.z])
        self.q_fp = np.array([fp.pose.pose.orientation.x, fp.pose.pose.orientation.y, fp.pose.pose.orientation.z, fp.pose.pose.orientation.w])
        self.v_fp = np.array([fp.twist.twist.linear.x, fp.twist.twist.linear.y, fp.twist.twist.linear.z])
        
        f1 = open(self.path+"est_fp_position_RN{}.txt".format(self.RN), 'a')
        f1.write("%s, %s, %s, %s, %s, %s\n" % (self.p_est[0], self.p_fp[0], self.p_est[1], self.p_fp[1],self.p_est[2], self.p_fp[2]))

        f2 = open(self.path+"est_fp_velocity_RN{}.txt".format(self.RN), 'a')
        f2.write("%s, %s, %s, %s, %s, %s\n" % (self.v_est[0], self.v_fp[0], self.v_est[1], self.v_fp[1],self.v_est[2], self.v_fp[2]))

        f3 = open(self.path+"est_fp_orientation_RN{}.txt".format(self.RN), 'a')
        f3.write("%s, %s, %s, %s, %s, %s, %s, %s\n" % (self.q_est[0], self.q_fp[0], self.q_est[1], self.q_fp[1],self.q_est[2], self.q_fp[2],self.q_est[3], self.q_fp[3]))

if __name__ == '__main__':
    name = 'udrone1'
    RN = rospy.get_param('run_number')
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('est_fp_comparison', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = compare(RN)
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

