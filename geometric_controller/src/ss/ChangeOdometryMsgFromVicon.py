#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thursday July 5th 2018
to get rid of covariance in odometry msg and (if that doesnt work) 
to make a 13x1 vector to send position, orientation, linear and angular 
velocities to the quad from vicon
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'


import rospy
import time
import numpy as np
from isy_geometric_controller.msg import modifiedodometry
from nav_msgs.msg import Odometry
import sys
import message_filters

class modified_odometry(object): 
    def __init__(self, name_of_uav): 
        self.uav = name_of_uav
        self.pub1 = rospy.Publisher('/hummingbird/modified/odom', modifiedodometry, queue_size = 1)
	self.pub2 = rospy.Publisher('/firefly/modified/odom', modifiedodometry, queue_size = 1)
        try:
            data1 = message_filters.Subscriber('/hummingbird/odom', Odometry, tcp_nodelay = True)
	    data2 = message_filters.Subscriber('/firefly/odom', Odometry, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([data1, data2], 100)
            ts.registerCallback(self.callback)
        except:
            print('problem subscribing to Odometry topic')
   
    def callback(self, data1, data2): 
	# hummingbird
        Msg1 = modifiedodometry()
        Msg1.header.stamp = data1.header.stamp
	Msg1.pose.position.x = data1.pose.pose.position.x
	Msg1.pose.position.y = data1.pose.pose.position.y
	Msg1.pose.position.z = data1.pose.pose.position.z
	Msg1.pose.orientation.x = data1.pose.pose.orientation.x
	Msg1.pose.orientation.y = data1.pose.pose.orientation.y
	Msg1.pose.orientation.z = data1.pose.pose.orientation.z
	Msg1.pose.orientation.w = data1.pose.pose.orientation.w
	Msg1.twist.linear.x = data1.twist.twist.linear.x
	Msg1.twist.linear.y = data1.twist.twist.linear.y
	Msg1.twist.linear.z = data1.twist.twist.linear.z
	Msg1.twist.angular.x = data1.twist.twist.angular.x
	Msg1.twist.angular.y = data1.twist.twist.angular.y
	Msg1.twist.angular.z = data1.twist.twist.angular.z
        self.pub1.publish(Msg1)

	# firefly
        Msg2 = modifiedodometry()
        Msg2.header.stamp = data2.header.stamp
	Msg2.pose.position.x = data2.pose.pose.position.x
	Msg2.pose.position.y = data2.pose.pose.position.y
	Msg2.pose.position.z = data2.pose.pose.position.z
	Msg2.pose.orientation.x = data2.pose.pose.orientation.x
	Msg2.pose.orientation.y = data2.pose.pose.orientation.y
	Msg2.pose.orientation.z = data2.pose.pose.orientation.z
	Msg2.pose.orientation.w = data2.pose.pose.orientation.w
	Msg2.twist.linear.x = data2.twist.twist.linear.x
	Msg2.twist.linear.y = data2.twist.twist.linear.y
	Msg2.twist.linear.z = data2.twist.twist.linear.z
	Msg2.twist.angular.x = data2.twist.twist.angular.x
	Msg2.twist.angular.y = data2.twist.twist.angular.y
	Msg2.twist.angular.z = data2.twist.twist.angular.z
        self.pub2.publish(Msg2)
if __name__ == '__main__':
    name = 'firefly'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('modified_odometry', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)

    try: 
        while not rospy.is_shutdown(): 
            c = modified_odometry(name)
            rospy.spin()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass


