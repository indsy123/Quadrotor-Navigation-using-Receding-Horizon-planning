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
from isy_geometric_controller.msg import Desired_Trajectory
from isy_geometric_controller.msg import modifiedodometry
from nav_msgs.msg import Odometry
import time 
import scipy
from scipy import special

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav, time_instance):
        self.time = time.time()
        self.counter = 0
        self.uav = name_of_uav
        self.pub = rospy.Publisher('/desired_trajectory', Desired_Trajectory, queue_size = 10, tcp_nodelay = True)
	self.T = 12
	self.w = 2*np.pi/self.T
        try:
            #rospy.Subscriber('/'+self.uav+'/odom', Odometry, self.callback, queue_size = 10, tcp_nodelay = True)
	    rospy.Subscriber('/'+self.uav+'/odometry_sensor1/odometry', Odometry, self.callback, queue_size = 100, tcp_nodelay = True)
	    #rospy.Subscriber('/'+self.uav+'/odom', Odometry, self.callback, queue_size = 100, tcp_nodelay = True)

        except:
            print('problem subscribing to odometry topic')


    
   
    def callback(self, data):
        #print self.time
        msg = Desired_Trajectory()
        msg.header.stamp = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
	t = time.time()
	tt = t-self.time

	if tt<=10: 
            msg.desired_position.x = -6.0#0.9 * np.cos(np.pi*(tt)/3)
	    msg.desired_position.y = 0#0.9 * np.sin(np.pi*(tt)/3)
	    msg.desired_position.z = 1

            msg.desired_velocity.x = 0#-0.9 * (np.pi/3) * np.sin(np.pi*(tt)/3)
	    msg.desired_velocity.y = 0#0.9 * (np.pi/3) * np.cos(np.pi*(tt)/3)
	    msg.desired_velocity.z = 0

            msg.desired_acceleration.x = 0#-0.9 * (np.pi/3)**2 * np.cos(np.pi*(tt)/3)
	    msg.desired_acceleration.y = 0#-0.9 * (np.pi/3)**2 * np.sin(np.pi*(tt)/3)
	    msg.desired_acceleration.z = 0.0

            msg.desired_direction.x = 1
            msg.desired_direction.y = 0
            msg.desired_direction.z = 0
        else:
	    msg.desired_position.x = -6.0
            msg.desired_position.y = 0.0 
	    msg.desired_position.z = 0.086

            msg.desired_velocity.x = 0.0
            msg.desired_velocity.y = 0.0                
	    msg.desired_velocity.z = 0

            msg.desired_acceleration.x = 0.0
            msg.desired_acceleration.y = 0.0
	    msg.desired_acceleration.z = 0

            msg.desired_direction.x = 1
            msg.desired_direction.y = 0
            msg.desired_direction.z = 0        
                


	msg.controller = 0 # position controller
        self.pub.publish(msg)
    
    """
    def callback(self, data):
        #print self.time
        msg = Desired_Trajectory()
        msg.header.stamp = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
	t = time.time()
	tt = t-self.time

	if tt<=3*self.T: 
            msg.desired_position.x = 1.0 * np.cos(self.w*tt)
	    msg.desired_position.y = 1.0 * 0.5* np.sin(2*self.w*tt)
	    msg.desired_position.z = 0.75 + 0.25*np.sin(self.w*tt)

            msg.desired_velocity.x = -1.0 * (self.w) * np.sin(self.w*tt)
	    msg.desired_velocity.y = 1.0*0.5 * (2*self.w) * np.cos(2*self.w*tt)
	    msg.desired_velocity.z =  0.25*self.w*np.cos(self.w*tt)

            msg.desired_acceleration.x = -1.0 * (self.w)**2 * np.cos(self.w*tt)
	    msg.desired_acceleration.y = -1.0*0.5 * (2*self.w)**2 * np.sin(2*self.w*tt)
	    msg.desired_acceleration.z = -0.25*self.w**2*np.sin(self.w*tt)
            msg.desired_direction.x = np.cos(2*self.w*tt)
            msg.desired_direction.y = np.sin(2*self.w*tt)
            msg.desired_direction.z = 0
        else:
	    msg.desired_position.x = 1.0
            msg.desired_position.y = 0.0 
	    msg.desired_position.z = 0.15

            msg.desired_velocity.x = 0.0
            msg.desired_velocity.y = 0.0                
	    msg.desired_velocity.z = 0
            msg.desired_acceleration.x = 0.0
            msg.desired_acceleration.y = 0.0
	    msg.desired_acceleration.z = 0
            msg.desired_direction.x = 1
            msg.desired_direction.y = 1
            msg.desired_direction.z = 0        

	msg.controller = 0 # position controller
        self.pub.publish(msg)
    """
   
# may get rid of the code below evntually when the trajectory topic will be 
# subscribed in the main controller script. Remember to initilize the 
# "Trajectory" node in controller script eventually.
if __name__ == '__main__':
    name = 'pelican'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('Trajectory', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    start_time = time.time()
    try: 
        while not rospy.is_shutdown(): 
            current_time = time.time()
            t = current_time-start_time
            #print t
            traj = trajectory(name, current_time)

            rospy.spin()
            #print 'a' , time.time()-a
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass
                

