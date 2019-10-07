#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'


import rospy,time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

class publish_goal(object): 
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher('/goal_to_chase', PointStamped, queue_size = 1, tcp_nodelay = True)
        #self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
        try:
            rospy.Subscriber('/turtlebot/ground_truth/state', Odometry, self.callback, tcp_nodelay = True)
        except:
            print('problem subscribing to one of the topic above')
   
    def callback(self, rt): 
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.point.x = rt.pose.pose.position.x
        msg.point.y = rt.pose.pose.position.y
        msg.point.z = rt.pose.pose.position.z
        #print rt.pose[40].position.x, rt.pose[40].position.y, rt.pose[40].position.z
        self.pub.publish(msg)
        #time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('publish_goal_to_chase', anonymous=False, log_level=rospy.DEBUG)
    freq = 100.0
    r = rospy.Rate(freq)
    
    try: 
        while not rospy.is_shutdown():             
            f = publish_goal()
            r.sleep()
    except rospy.ROSInterruptException(): 
        pass
