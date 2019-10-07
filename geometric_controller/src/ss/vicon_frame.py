#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  12 13:30:01 2017
This is the the script for correctly setting old system vicon frame
you need to make frame as shown in the instruction for any configuration 
you have and then run Upenn's package "vicon" to get odometry data of the frame
and location of each marker.
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'


import rospy
import time
import numpy as np
#from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
#from tf import TransformListener, TransformerROS
#from nav_msgs.msg import Odometry
#from pyquaternion import Quaternion 
from parameters import parameters
from isy_geometric_controller.msg import control_inputs
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from mav_msgs.msg import Actuators
from numpy import sin as sin
from numpy import cos as cos 
from numpy import pi as pi 
#from numpy import transpose as tp # just to reduce typing 

class vicon_frame(object): 
    def __init__(self, name_of_uav,parameters): 
        self.uav = name_of_uav
        self.d = parameters[0]
        self.tau_m = parameters[1]
        self.tau_f = parameters [2]
        self.pub = rospy.Publisher('/'+self.uav+'/odom', Actuators, queue_size = 100)
        try:
            rospy.Subscriber('/control_inputs', control_inputs, self.callback, queue_size = 100, tcp_nodelay = False)
        except:
            print('problem subscribing to topic "/control_inputs"')
   
    def callback(self, data): 
        f = data.Total_thrust; M1 = data.Moment_x
        M2 = data.Moment_y; M3 = data.Moment_z
        T = tp(ar([[M1,M2,M3,f]]))
        if self.uav == 'pelican'or self.uav == 'hummingbird':       
            c1 = tp(ar([[0, -self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c2 = tp(ar([[self.d*self.tau_f, 0, -self.tau_f*self.tau_m, self.tau_f]]))
            c3 = tp(ar([[0, self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c4 = tp(ar([[-self.d*self.tau_f, 0, -self.tau_f*self.tau_m, self.tau_f]]))
            C = np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2
            w_square = np.dot(np.linalg.inv(C),T)
            w = np.sqrt(np.abs(w_square))
            Msg = Actuators()
            #Msg.header.stamp = rospy.Time.now()
	    Msg.header.stamp = data.header.stamp
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]
            f2 = open('motorspeeds.txt', 'a')
            f2.write("%s, %s, %s, %s\n" % (w[0][0], w[1][0], w[2][0], w[3][0]))
            self.pub.publish(Msg)
        else: 
            c1 = tp(ar([[sin(pi/6)*self.d*self.tau_f, -cos(pi/6)*self.d*self.tau_f, -self.tau_f*self.tau_m, self.tau_f]]))
            c2 = tp(ar([[sin(pi/2)*self.d*self.tau_f, -cos(pi/2)*self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c3 = tp(ar([[sin(5*pi/6)*self.d*self.tau_f, -cos(5*pi/6)*self.d*self.tau_f, -self.tau_f*self.tau_m, self.tau_f]]))
            c4 = tp(ar([[sin(7*pi/6)*self.d*self.tau_f, -cos(7*pi/6)*self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c5 = tp(ar([[sin(3*pi/2)*self.d*self.tau_f, -cos(3*pi/2)*self.d*self.tau_f, -self.tau_f*self.tau_m, self.tau_f]]))
            c6 = tp(ar([[sin(11*pi/6)*self.d*self.tau_f, -cos(11*pi/6)*self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            C = np.column_stack((c1,c2,c3,c4,c5,c6)) # solving linear eq T = Cw^2 to get w^2
            inverted_matrix = np.dot(C.T, np.linalg.inv(np.dot(C, C.T)))
            w_square = np.dot(inverted_matrix,T)
            w = np.sqrt(np.abs(w_square))
            Msg = Actuators()
            Msg.header.stamp = rospy.Time.now()
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0], w[4][0], w[5][0]]
            self.pub.publish(Msg)
if __name__ == '__main__':
    name = 'hummingbird'
    rospy.init_node('vicon_frame', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    try: 
        while not rospy.is_shutdown(): 
            c = vicon_frame(name)
            rospy.spin()
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass


