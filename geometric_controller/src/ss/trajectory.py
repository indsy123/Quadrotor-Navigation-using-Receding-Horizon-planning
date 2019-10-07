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
import time 
import scipy
from scipy import special

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self,time_instance):
        self.time = time_instance
        self.counter = 0
        self.pub = rospy.Publisher('/desired_trajectory', Desired_Trajectory, queue_size = 100, tcp_nodelay = True)
    """
    def PublishMsg(self):
        msg = Desired_Trajectory()
        msg.header.stamp = rospy.Time.now()
        msg.desired_position.x = 1.0*np.cos(0.025*np.pi*self.time)
        msg.desired_position.y = 1.0*np.sin(0.025*np.pi*self.time)
        msg.desired_position.z = 2#1*self.time#1.0 + 0.05 * np.cos(np.pi*self.time)
                
        msg.desired_velocity.x = -(1.0*np.pi*0.025) * np.sin(0.025*np.pi*self.time)
        msg.desired_velocity.y = (1.0*np.pi*0.025) * np.cos(0.025*np.pi*self.time)
        msg.desired_velocity.z = 0#0.1#0.0#-0.05 * np.pi * np.sin(np.pi*self.time)
        
        msg.desired_acceleration.x = -1.0*(1*np.pi*0.025)**2 * np.cos(0.025*np.pi*self.time)
        msg.desired_acceleration.y = -1.0*(1*np.pi*0.025)**2 * np.sin(0.025*np.pi*self.time)
        msg.desired_acceleration.z = 0#0.0#-0.05 * np.pi**2 * np.cos(np.pi*self.time)
                
        msg.desired_direction.x = 1#np.cos(np.pi*self.time)
        msg.desired_direction.y = 1#np.sin(np.pi*self.time)
        msg.desired_direction.z = 0.0
        
        msg.desired_direction_dot.x = 0#-(np.pi/10)*np.sin(np.pi*self.time/10)
        msg.desired_direction_dot.y = 0#(np.pi/10)*np.cos(np.pi*self.time/10)
        msg.desired_direction_dot.z = 0.0
        
        msg.desired_direction_ddot.x = 0
        msg.desired_direction_ddot.y = 0
        msg.desired_direction_ddot.z = 0
        self.pub.publish(msg)
    """    
    """
    def PublishMsg(self):
        
        msg = Desired_Trajectory()
        msg.header.stamp = rospy.Time.now()
        msg.desired_position.y = 1.0 * np.sin(np.pi*self.time/6)
        msg.desired_position.x = 1.0 * np.cos(np.pi*self.time/6)
        #if 1+self.time <= 2: 
        msg.desired_position.z =  1
        #else: 111
        #    msg.desired_position.z = 2
        #    msg.desired_velocity.z = 0.0
                
        msg.desired_velocity.y = 1.0 * (np.pi/6) * np.cos(np.pi*self.time/6)
        msg.desired_velocity.x = -1.0 * (np.pi/6) * np.sin(np.pi*self.time/6)
        msg.desired_velocity.z = 0.0
                
        msg.desired_acceleration.x = -1.0 * (np.pi/6)**2 * np.cos(np.pi*self.time/6)
        msg.desired_acceleration.y = -1.0 * (np.pi/6)**2 * np.sin(np.pi*self.time/6)
        msg.desired_acceleration.z = 0.0
                
        #msg.desired_direction.x = 0
        #if self.time<=3:
        msg.desired_direction.x = 1#np.cos(np.pi*self.time/60)
        msg.desired_direction.y = 1#np.sin(np.pi*self.time/60) 
        msg.desired_direction_dot.x = 0#-(np.pi/6)*np.sin(np.pi*self.time/6) 
        msg.desired_direction_dot.y = 0#(np.pi/6)*np.cos(np.pi*self.time/6) 
        msg.desired_direction_ddot.x = 0#-(np.pi/6)**2*np.cos(np.pi*self.time/6) 
        msg.desired_direction_ddot.y = 0#-(np.pi/6)**2*np.sin(np.pi*self.time/6) 
            
        msg.desired_direction.z = 0
        msg.desired_direction_dot.z = 0
        msg.desired_direction_ddot.z = 0
        self.pub.publish(msg)
    """
    
    
    def PublishMsg(self):
        #print self.time
        msg = Desired_Trajectory()
        msg.header.stamp = rospy.Time.now()

        
        msg.desired_position.x = 0
        msg.desired_position.y = 0
	if self.time<=5:
	    msg.desired_position.z = 1
	else: 
	    msg.desired_position.z = 0.08
        
        msg.desired_velocity.z = 0.0        
        msg.desired_velocity.x = 0.0#0.5
        msg.desired_velocity.y = 0.0
        #msg.desired_velocity.z = 0.1
                
        msg.desired_acceleration.x = 0.0
        msg.desired_acceleration.y = 0.0
        msg.desired_acceleration.z = 0.0
                
        msg.desired_direction.x = 1
        msg.desired_direction.y = 0
        msg.desired_direction.z = 0
        msg.desired_direction_dot.x = 0.0
        msg.desired_direction_dot.y = 0.0
        msg.desired_direction_dot.z = 0.0
        
        msg.desired_direction_ddot.x = 0.0
        msg.desired_direction_ddot.y = 0.0
        msg.desired_direction_ddot.z = 0.0
        self.pub.publish(msg)
    
    """
    def PublishMsg(self):
        msg = Desired_Trajectory()
        msg.header.stamp = rospy.Time.now()



        if self.time <= 2 :
            msg.desired_velocity.z = 0.5#np.sin(self.time*np.pi)     

        else: 
            msg.desired_velocity.z = 0.0
        #a = scipy.special.erf(0.5*self.time)    
        #msg.desired_velocity.z = 0.5    
        msg.desired_velocity.x = 0.0
        msg.desired_velocity.y = 0.0                
        
        msg.desired_acceleration.x = 0.0
        msg.desired_acceleration.y = 0.0
        msg.desired_acceleration.z = 0.0

        msg.desired_jerk.x = 0.0
        msg.desired_jerk.y = 0.0
        msg.desired_jerk.z = 0.0
                
        msg.desired_direction.x = np.cos(45*np.pi/180)
        msg.desired_direction.y = np.sin(45*np.pi/180)
        msg.desired_direction.z = 0

        msg.desired_direction_dot.x = 0.0
        msg.desired_direction_dot.y = 0.0
        msg.desired_direction_dot.z = 0.0
        
        msg.desired_direction_ddot.x = 0.0
        msg.desired_direction_ddot.y = 0.0
        msg.desired_direction_ddot.z = 0.0
        msg.controller = 1
        self.pub.publish(msg)
        self.counter = self.counter+1
    """
    
# may get rid of the code below evntually when the trajectory topic will be 
# subscribed in the main controller script. Remember to initilize the 
# "Trajectory" node in controller script eventually.
if __name__ == '__main__':
    rospy.init_node('Trajectory', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    start_time = time.time()
    try: 
        while not rospy.is_shutdown(): 
            current_time = time.time()
            t = current_time-start_time
            #print t
            traj = trajectory(t)
            traj.PublishMsg()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass
                

