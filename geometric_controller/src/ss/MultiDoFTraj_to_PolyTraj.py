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
import time 
import scipy
import std_msgs.msg
from scipy import special
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometric_controller.msg import PolynomialTrajectory

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
        self.uav = name_of_uav
        self.pub = rospy.Publisher('/polynomial_trajectory', PolynomialTrajectory, queue_size  = 100, tcp_nodelay = True)
        try:
            rospy.Subscriber('/firefly1/polynomial_trajectory', MultiDOFJointTrajectory, self.callback, queue_size = 1, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    
    def callback(self, traj):
        msg = PolynomialTrajectory()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        msg.header = header

        for i in range(len(traj.points)): 
            #time.sleep(0.0667/20)
            msg.pdes.x = traj.points[i].transforms[0].translation.x
            msg.pdes.y = traj.points[i].transforms[0].translation.y
            msg.pdes.z = traj.points[i].transforms[0].translation.z
            
            msg.vdes.x = traj.points[i].velocities[0].linear.x
            msg.vdes.y = traj.points[i].velocities[0].linear.y
            msg.vdes.z = traj.points[i].velocities[0].linear.z
            
            msg.ades.x = traj.points[i].accelerations[0].linear.x
            msg.ades.y = traj.points[i].accelerations[0].linear.y
            msg.ades.z = traj.points[i].accelerations[0].linear.z
            
            msg.ddes.x = traj.points[i].accelerations[0].angular.x
            msg.ddes.y = traj.points[i].accelerations[0].angular.y
            msg.ddes.z = traj.points[i].accelerations[0].angular.z
            # "time_from_start" in MultiDOFJointTrajectory is rospy.Duration, to_sec() is needed to convert that to seconds 
            tt = traj.points[i].time_from_start.to_sec() 
            msg.controller = 0
            self.pub.publish(msg) 
	 
if __name__ == '__main__':
    name = 'diy'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('trajectory_converter_node', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = trajectory(name)
    try: 
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

