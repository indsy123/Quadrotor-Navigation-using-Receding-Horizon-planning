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
from scipy import special

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
        open("vicon.txt", "w").close()
	open("t265_odom.txt", "w").close()
        self.uav = name_of_uav
        try:
            #data = rospy.Subscriber('/t265/odom/sample', Odometry, self.callback, queue_size = 10, tcp_nodelay = True)
	    data = rospy.Subscriber('/diy2/odom', Odometry, self.callback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    
    def callback(self, data):

        t = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
	x = data.pose.pose.position.x; y = data.pose.pose.position.y; z = data.pose.pose.position.z
	qx = data.pose.pose.orientation.x; qy = data.pose.pose.orientation.y; qz = data.pose.pose.orientation.z; qw = data.pose.pose.orientation.w;
	vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z
	wx = data.twist.twist.angular.x; wy = data.twist.twist.angular.y; wz = data.twist.twist.angular.z
	f1 = open('vicon.txt', 'a')
	f1.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (t, x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz))
	#f2 = open("t265_odom.txt", "a")
	#f2.write("%s, %s, %s, %s, %s, %s, %s\n" % (x, y, z, qx, qy, qz, qw))

   
# may get rid of the code below evntually when the trajectory topic will be 
# subscribed in the main controller script. Remember to initilize the 
# "Trajectory" node in controller script eventually.
if __name__ == '__main__':
    name = 'diy2'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('vicon_odom', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    traj = trajectory(name)
    try: 
        while not rospy.is_shutdown():            
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

