#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 28 10:24:25 2019
test if it is possible to subscribe to a topic intermittently 
@author: ind
"""



import time
import numpy as np
from pyquaternion import Quaternion as Qt
import rospy
import std_msgs.msg
from nav_msgs.msg import Odometry 


class intermittent_callback(object): 
    def __init__(self): 
        """initialization"""
        self.position = np.array([0,0,0])
        self.t = time.time()

        try:
            self.sub = rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.odometry_callback, queue_size = 1, tcp_nodelay = True)
            rospy.spin()
        except:
            print('problem subscribing to odometry topic')
    
    def get_flag(self, time):
        """generates a flag based on time """
        if time >=1.0 and time<=2.0: 
            subscribe = False
        else: 
            subscribe = True
        return subscribe

    def odometry_callback(self, data):
        """get the odometry of the quadrotor"""
        tt = time.time()-self.t        
        subscribe = self.get_flag(tt)
        if subscribe == True: 
            self.position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
            f1 = open('subscribing_check.txt', 'a')
            f1.write('%s, %s, %s, %s\n' %(tt, self.position[0], self.position[1], self.position[2]))
        else: 
            self.sub.unregister()




# if python says run, then we should run
if __name__ == '__main__':
    rospy.init_node('subscribing_check', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(100)    
    intermittent_callback()
    #try:
    #    while not rospy.is_shutdown(): 
    #        rospy.spin()
    #except rospy.ROSInterruptException(): 
    #    pass
    
