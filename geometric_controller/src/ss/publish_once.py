#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 09:53:45 2019
Note to publish just once on a pose topic
@author: indrajeet
"""
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

class PoseChange(object):
    def __init__(self): 
        self.msg = PoseStamped()
        
        rospy.Subscriber('/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('/initialpose', PoseStamped, queue_size = 1)
        
    def callback(self, data):
        self.msg.header = Header()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "map"
        self.msg.pose.position = data.pose.position
        
    def run(self): 
        self.pub.publish(self.msg)
        
if __name__ == '__main__':
    name = 'firefly'
    rospy.init_node("pose_change", anonymous=True, log_level=rospy.DEBUG)
    rate = rospy.Rate(10)
    try:
        m = PoseChange()       
        while not rospy.is_shutdown():
            connections = m.pub.get_num_connections()
            #rospy.loginfo('connections: %d', connections)
            if connections > 0: 
                m.run()
                rospy.loginfo('Published')
                break
            rate.sleep()
    except rospy.ROSInterruptException(): 
        raise e

