#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Publish camera info fro two pointgrey cameras
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
import rospy, yaml
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
import time 
import scipy
from scipy import special

class publish_cam_info(object): 
    "calculates desired position, linear velocity, linear acceleration and direction"
    def __init__(self, name_of_uav):
        self.uav = name_of_uav
        self.pub = rospy.Publisher('/camera1/camera_info', CameraInfo, queue_size = 1, tcp_nodely = False)
       
        try:
            rospy.Subscriber('/camera1/image_raw', self.callback1, queue_size = 1, tcp_nodelay = True)
            
        except:
            print('problem subscribing to one of the camera topics')


    
    def callback1(self, data):
        """publish camera info"""
        msg = CameraInfo()
        msg.header.stamp = data.header.stamp
        
        x = data.pose.pose.position.x; y = data.pose.pose.position.y; z = data.pose.pose.position.z
        qx = data.pose.pose.orientation.x; qy = data.pose.pose.orientation.y; qz = data.pose.pose.orientation.z; qw = data.pose.pose.orientation.w;
        vx = data.twist.twist.linear.x; vy = data.twist.twist.linear.y; vz = data.twist.twist.linear.z
        wx = data.twist.twist.angular.x; wy = data.twist.twist.angular.y; wz = data.twist.twist.angular.z


if __name__ == '__main__':
    name = 'diy'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node('publish_camera_info', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(30)
    publish_cam_info()
    try: 
        while not rospy.is_shutdown(): 
            rospy.spin()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass
                

