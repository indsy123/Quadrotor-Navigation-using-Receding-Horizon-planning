#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'


import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
#from tf import TransformListener, TransformerROS
#from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Imu
from pyquaternion import Quaternion 
#from parameters import parameters
#from isy_geometric_controller.msg import Desired_Trajectory, control_inputs
from gazebo_msgs.msg import LinkStates
#import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
from tf import transformations




class move_turtlebot(object): 
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 1)
        #self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
        #try:
        #    odom = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback, tcp_nodelay = True)
        #    #rospy.spin()
        ##except:
        #    print('problem subscribing to one of the topic above')
   
    def callback(self): 
        Msg = Twist()
        if self.counter <=1200: 
            v = 1.2
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = 0.0; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = -0.1;
        elif self.counter <=1900: 
            v = 1.2
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = 0; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = -0.35;   
        else: 
            v = 1.2
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = 0; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = 0.4;            
        self.pub.publish(Msg)
        self.counter = self.counter+1

    def callback_basic_R2(self): 
        Msg = Twist()
        if self.counter <=600: 
            v = 1.2
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = 0.0; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = -0.05;
        elif self.counter > 600 and self.counter <=1150: 
            v = 1.8
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = 0; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = -0.47;   
        elif self.counter > 1150 and self.counter <= 1550: 
            v = 1.5
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = -0.1; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = 0.55;  
        else: 
            v = 1.5
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = -0.1; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = 0.45;
        self.pub.publish(Msg)
        self.counter = self.counter+1
 
    def callback_basic_R1(self): 
        Msg = Twist()
        if self.counter <=3000: 
            v = 1.03
            #print odom.pose[40].position.x # it works
            Msg.linear.x = v; Msg.linear.y = 0.0; Msg.linear.z = 0.0
            Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = 0.018;
        #elif self.counter > 600 and self.counter <=1150: 
        #    v = 1.8
        #    #print odom.pose[40].position.x # it works
        #    Msg.linear.x = v; Msg.linear.y = 0; Msg.linear.z = 0.0
        #    Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = -0.63;   
        #elif self.counter > 1150 and self.counter <= 1500: 
        #    v = 1.5
        #    #print odom.pose[40].position.x # it works
        #    Msg.linear.x = v; Msg.linear.y = -0.1; Msg.linear.z = 0.0
        #    Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = 0.65;  
        #else: 
        #    v = 1.5
        #    #print odom.pose[40].position.x # it works
        #    Msg.linear.x = v; Msg.linear.y = -0.1; Msg.linear.z = 0.0
        #    Msg.angular.x = 0.0; Msg.angular.y = 0.0; Msg.angular.z = 0.5;
        self.pub.publish(Msg)
        self.counter = self.counter+1       
if __name__ == '__main__':
    rospy.init_node('move_turtlebot', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    c = move_turtlebot()
    try: 
        while not rospy.is_shutdown():  
            c.callback_basic_R2()
            r.sleep()
    except rospy.ROSInterruptException(): 
        pass

