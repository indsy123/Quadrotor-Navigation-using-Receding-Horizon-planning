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
from gazebo_msgs.msg import ModelState

class trajectory(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
        self.uav = name_of_uav
        self.no = 1
        self.y = 0; self.yy = 0
        self.pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 100, tcp_nodelay = True)
        self.pub2 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 100, tcp_nodelay = True)
        self.pub3 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 100, tcp_nodelay = True)
        self.pub4 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 100, tcp_nodelay = True)
        self.pub5 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 100, tcp_nodelay = True)
        self.pub6 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 100, tcp_nodelay = True)
        try:
            rospy.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, self.callback, tcp_nodelay = True)
        except:
            print('problem subscribing to odometry topic')


    
    def callback(self, data):

        t = data.header.stamp.to_sec()
        msg1 = ModelState()
        msg2 = ModelState()
        vel = -0.5
        dt = 0.01
        self.y = self.y + vel*dt
        
        msg1.model_name = "unit_box_11"
        msg1.pose.position.x = self.y; msg1.pose.position.y = 0; msg1.pose.position.z = 2.5
        msg1.pose.orientation.x = 0; msg1.pose.orientation.y = 0; msg1.pose.orientation.z = 0; msg1.pose.orientation.w = 1
        msg1.twist.linear.x = 0; msg1.twist.linear.y = vel; msg1.twist.linear.z = 0
        msg1.twist.angular.x = 0; msg1.twist.angular.y = 0.0; msg1.twist.angular.z = 0.0
        msg1.reference_frame = "world"
        self.pub1.publish(msg1)

        msg1.model_name = "unit_box_4"
        msg1.pose.position.z = 1
        self.pub2.publish(msg1)
        msg1.model_name = "unit_box_7"
        msg1.pose.position.z = 1.5
        self.pub3.publish(msg1)
        msg1.model_name = "unit_box_8"
        msg1.pose.position.z = 1
        self.pub4.publish(msg1)  
        
        
       
        vel = -0.5
        dt = 0.01
        self.yy = self.yy + vel*dt
        msg2.model_name = "unit_box_5"
        msg2.pose.position.x = 0; msg2.pose.position.y = self.yy; msg2.pose.position.z = 2
        msg2.pose.orientation.x = 0; msg2.pose.orientation.y = 0; msg2.pose.orientation.z = 0; msg2.pose.orientation.w = 1
        msg2.twist.linear.x = 0; msg2.twist.linear.y = vel; msg2.twist.linear.z = 0
        msg2.twist.angular.x = 0; msg2.twist.angular.y = 0.0; msg2.twist.angular.z = 0.0
        msg2.reference_frame = "world"
        self.pub5.publish(msg2)
        
        msg2.model_name = "unit_box_12"
        msg2.pose.position.z = 1.5
        self.pub6.publish(msg2)  
        
# may get rid of the code below evntually when the trajectory topic will be 
# subscribed in the main controller script. Remember to initilize the 
# "Trajectory" node in controller script eventually.
if __name__ == '__main__':
    name = 'firefly'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('set_gazebo_model_state', anonymous=False, log_level=rospy.DEBUG)

    traj = trajectory(name)
    try: 
        while not rospy.is_shutdown(): 
            
            rospy.spin()

    except rospy.ROSInterruptException(): 
        pass
                

