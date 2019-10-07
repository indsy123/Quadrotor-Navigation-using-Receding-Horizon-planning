#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  12 13:30:01 2017
This is the main controller as proposed by T Lee in his paper: 
"Geomentric tracking control of a quadrotor in SE(3)"
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

class motor_speeds(object): 
    def __init__(self, name_of_uav,parameters): 
        self.uav = name_of_uav
        self.d = parameters[0]
        self.tau_m = parameters[1]
        self.tau_f = parameters [2]
	self.w0_h = 1450*2*np.pi/60 # initial rotational speed (rad/sec) for hummingbird blades corresponding to the initial speed of the motors
	self.w0_f = 1890*2*np.pi/60 # initial rotational speed (rad/sec) for firefly blades corresponding to the initial speed of the motors
	self.w0_p = 1450*2*np.pi/60 # intial rotational speed of the pelican 
        self.pub = rospy.Publisher('/'+self.uav+'/command/motor_speed', Actuators, queue_size = 10)
        try:
            rospy.Subscriber('/control_inputs', control_inputs, self.callback, queue_size = 10, tcp_nodelay = True)
        except:
            print('problem subscribing to topic "/control_inputs"')
   
    def callback(self, data): 
        f = data.Total_thrust; M1 = data.Moment_x
        M2 = data.Moment_y; M3 = data.Moment_z
        T = tp(ar([[M1,M2,M3,f]]))
        Msg = Actuators()
        #Msg.header.stamp = rospy.Time.now()
	Msg.header.stamp = data.header.stamp
	if self.uav == 'hummingbird':
	    #self.tau_fh = 8.54858e-6
	    c1 = tp(ar([[0, -self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c2 = tp(ar([[self.d*self.tau_f, 0, -self.tau_f*self.tau_m, self.tau_f]]))
            c3 = tp(ar([[0, self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c4 = tp(ar([[-self.d*self.tau_f, 0, -self.tau_f*self.tau_m, self.tau_f]]))
            C = np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2 
            w_square = np.dot(np.linalg.inv(C),T)
	    w_initial = np.array([[self.w0_h**2], [self.w0_h**2], [self.w0_h**2], [self.w0_h**2]])
            w = np.sqrt(np.abs(w_square+w_initial))
            w[w>800.0] = 800.0
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]
	    #print w_square, w

            self.pub.publish(Msg)
	elif self.uav== 'pelican': 
	    #self.tau_fp = 9.986e-6 # for the time being using eth value, need to find a good solution
	    c1 = tp(ar([[0, -self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c2 = tp(ar([[self.d*self.tau_f, 0, -self.tau_f*self.tau_m, self.tau_f]]))
            c3 = tp(ar([[0, self.d*self.tau_f, self.tau_f*self.tau_m, self.tau_f]]))
            c4 = tp(ar([[-self.d*self.tau_f, 0, -self.tau_f*self.tau_m, self.tau_f]]))
            C = np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2 
            w_square = np.dot(np.linalg.inv(C),T)
	    #w_initial = np.array([[self.w0_p**2], [self.w0_p**2], [self.w0_p**2], [self.w0_p**2]])
            #w = np.sqrt(np.abs(w_square+w_initial))
            w = np.sqrt(np.abs(w_square))
            w[w>800.0] = 800.0
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
	    #w_initial = np.array([[self.w0_f**2], [self.w0_f**2], [self.w0_f**2], [self.w0_f**2], [self.w0_f**2], [self.w0_f**2]])
            #w = np.sqrt(np.abs(w_square+w_initial))
            w = np.sqrt(np.abs(w_square))
            #Msg = Actuators()
            #Msg.header.stamp = rospy.Time.now()
            w[w>900.0] = 900.0
            Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0], w[4][0], w[5][0]]
	    f2 = open('motorspeeds.txt', 'a')
            f2.write("%s, %s, %s, %s, %s, %s\n" % (w[0][0], w[1][0], w[2][0], w[3][0], w[4][0], w[5][0]))
            self.pub.publish(Msg)
if __name__ == '__main__':
    name = 'pelican'
    #name = rospy.get_param('~vehicle_name')
    rospy.init_node(name+'_motor_speeds', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(200)
    p = parameters(name)
    parameter_list = [p.CG_to_CoR(), p.tau_m(), p.tau_t()]

    try: 
        while not rospy.is_shutdown(): 
            #a = time.time()
            c = motor_speeds(name, parameter_list)
            rospy.spin()
            #print 'a' , time.time()-a
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass


