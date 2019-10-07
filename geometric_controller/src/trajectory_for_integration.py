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
from geometric_controller.msg import Desired_Trajectory
from geometric_controller.msg import modifiedodometry
from nav_msgs.msg import Odometry
import time 
import scipy
from scipy import special

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
        self.time = time.time()
        self.counter = 0
        self.uav = name_of_uav
        self.pub = rospy.Publisher('/desired_trajectory', Desired_Trajectory, queue_size = 100, tcp_nodelay = True)
        self.T = 5
        self.w = 2*np.pi/self.T
        try:
            #rospy.Subscriber('/'+self.uav+'/odom', Odometry, self.callback, queue_size = 10, tcp_nodelay = True)
            rospy.Subscriber('/'+self.uav+'/odometry_sensor1/odometry', Odometry, self.callback, queue_size = 100, tcp_nodelay = True)
            #rospy.Subscriber('/'+self.uav+'/odom', Odometry, self.callback, queue_size = 100, tcp_nodelay = True)

        except:
            print('problem subscribing to odometry topic')


    
    """
    def callback(self, data):
        #print self.time
        msg = Desired_Trajectory()
        msg.header.stamp = data.header.stamp
        #msg.header.stamp = rospy.Time.now()
        t = time.time()
        tt = t-self.time
        if tt<=50: 
            msg.desired_position.x = 2.0 * np.cos(np.pi*(tt)/3)
            msg.desired_position.y = 2.0 * np.sin(np.pi*(tt)/3)
            msg.desired_position.z = 1

            msg.desired_velocity.x = -2.0 * (np.pi/3) * np.sin(np.pi*(tt)/3)
            msg.desired_velocity.y = 2.0 * (np.pi/3) * np.cos(np.pi*(tt)/3)
            msg.desired_velocity.z = 0

            msg.desired_acceleration.x = -2.0 * (np.pi/3)**2 * np.cos(np.pi*(tt)/3)
            msg.desired_acceleration.y = -2.0 * (np.pi/3)**2 * np.sin(np.pi*(tt)/3)
            msg.desired_acceleration.z = 0.0

            msg.desired_direction.x = np.cos(np.pi*(tt)/10)
            msg.desired_direction.y = np.sin(np.pi*(tt)/10)
            msg.desired_direction.z = 0
        else:
            msg.desired_position.x = 0.0
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
        msg = Desired_Trajectory()
        msg.header.stamp = data.header.stamp
        tt = time.time()-self.time
        t0 = 0#self.T
        
        #if tt<=initial_period:
            
        #if tt <=self.T: 
        #    msg.pdes.x = 1.5; msg.pdes.y = 0; msg.pdes.z = 1
        #    msg.vdes.x = 0; msg.vdes.y = 0; msg.vdes.z = 0
        #    msg.ades.x = 0; msg.ades.y = 0; msg.ades.z = 0
        #    msg.jdes.x = 0; msg.jdes.y = 0; msg.jdes.z = 0
        #    msg.ddes.x = 1; msg.ddes.y = 0; msg.ddes.z = 0
        #    msg.ddes_dot.x = 0; msg.ddes_dot.y = 0; msg.ddes_dot.z = 0
            
        if tt<=20*self.T:
            msg.pdes.x = 1.5 * np.cos(self.w*(tt-t0))
            
            #msg.pdes.y = 1.5 * np.sin(2*self.w*tt)
            msg.pdes.y = 1.5 * np.sin(self.w*(tt-t0))
            msg.pdes.z = 1.0# + 0.25*np.sin(self.w*(tt-t0))
            self.previous_position = np.array([msg.pdes.x, msg.pdes.y, msg.pdes.z])

            msg.vdes.x = -1.5 * (self.w) * np.sin(self.w*(tt-t0))
            #msg.vdes.y = 1.5* (2*self.w) * np.cos(2*self.w*tt)
            msg.vdes.y = 1.5* (self.w) * np.cos(self.w*(tt-t0))
            msg.vdes.z =  0#0.25*self.w*np.cos(self.w*(tt-t0))

            msg.ades.x = -1.5 * (self.w)**2 * np.cos(self.w*(tt-t0))
            #msg.ades.y = -1.5 * (2*self.w)**2 * np.sin(2*self.w*tt)
            msg.ades.y = -1.5 * (self.w)**2 * np.sin(self.w*(tt-t0))
            msg.ades.z = 0#-0.25*self.w**2*np.sin(self.w*(tt-t0))

            msg.jdes.x = 1.5 * (self.w)**3 * np.sin(self.w*(tt-t0))
            #msg.ades.y = -1.5 * (2*self.w)**2 * np.sin(2*self.w*tt)
            msg.jdes.y = -1.5 * (self.w)**3 * np.cos(self.w*(tt-t0))
            msg.jdes.z = 0#-0.25 * self.w**3 * np.cos(self.w*(tt-t0))
            
            msg.ddes.x = np.cos(self.w*(tt-t0))
            msg.ddes.y = np.sin(self.w*(tt-t0))
            msg.ddes.z = 0
            
            msg.ddes_dot.x = -self.w*np.sin(self.w*(tt-t0))
            msg.ddes_dot.y = self.w*np.cos(self.w*(tt-t0))
            msg.ddes_dot.z = 0
        else:
            msg.pdes.x = self.previous_position[0]
            msg.pdes.y = self.previous_position[1] 
            msg.pdes.z = 0.15

            msg.vdes.x = 0.0
            msg.vdes.y = 0.0                
            msg.vdes.z = 0
            
            msg.ades.x = 0.0
            msg.ades.y = 0.0
            msg.ades.z = 0
            
            msg.ddes.x = 1
            msg.ddes.y = 0
            msg.ddes.z = 0   
            
            msg.ddes_dot.x = 0
            msg.ddes_dot.y = 0
            msg.ddes_dot.z = 0
            
            msg.controller = 0 # position controller
        self.pub.publish(msg)

if __name__ == '__main__':
    name = 'firefly'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('Trajectory', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    start_time = time.time()
    try: 
        while not rospy.is_shutdown(): 
            traj = trajectory(name)
            rospy.spin()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass
                

