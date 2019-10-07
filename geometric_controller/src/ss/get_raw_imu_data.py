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
from sensor_msgs.msg import Imu
import time 


class raw_imu_data(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self):
	self.start_time = time.time()
        try:
            rospy.Subscriber('/mavros/imu/data_raw', Imu, self.callback, queue_size = 1, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    
    def callback(self, data):

        t = time.time()-self.start_time
        wx = data.angular_velocity.x; wy = data.angular_velocity.y; wz = data.angular_velocity.z 
        ax = data.linear_acceleration.x; ay = data.linear_acceleration.y; az = data.linear_acceleration.z
        f = open('vectornav_imu_raw.txt', 'a')
        f.write("%s, %s, %s, %s, %s, %s, %s\n" % (t, wx, wy, wz, ax, ay, az))

   
if __name__ == '__main__':
    rospy.init_node('get_raw_imu', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    imu = raw_imu_data()
    try: 
        while not rospy.is_shutdown(): 
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
                

