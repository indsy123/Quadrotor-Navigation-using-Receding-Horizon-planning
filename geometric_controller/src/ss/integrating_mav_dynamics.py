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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion 
from parameters import parameters
from geometric_controller.msg import control_inputs
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
from numpy import sin as sin
from numpy import cos as cos
from numpy import pi as pi 
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators
from geometric_controller.msg import PolynomialTrajectory, velocities_from_navigation_function
#from scipy.interpolate import splrep, splev

class integrating_mav_dynamics(object): 
    def __init__(self, name_of_uav,parameters): 
        self.uav = name_of_uav
        self.m = parameters[0]
        self.J = parameters[1]
        self.Jinv = np.linalg.inv(self.J)
        self.kx = parameters[2]
        self.kv = parameters[3]
        self.kR = parameters[4]
        self.g = parameters[5]
        self.kOmega = parameters[6]
        self.d = parameters[7]
        self.tau_m = parameters[8]
        self.tau_t = parameters[9]
        self.e = parameters[10]
        self.counter = 0
        self.kR_normalized = np.dot(np.linalg.inv(self.J), self.kR)
        self.kOmega_normalized = np.dot(np.linalg.inv(self.J), self.kOmega)
        self.time_points = []; self.time = time.time()
        self.smooth_vx = []; self.smooth_vy = []; self.smooth_vz = []
        self.dt = 1.0/200 # time step based on 150Hz frequency
        self.initial_condition_counter = 0
        open("forces_using_motor_speed.txt", "w").close()
        open("components_in_moments.txt", "w").close()
        open("angular_velocity_comparison.txt", "w").close()
        open("linear_velocity_comparison.txt", "w").close()
        open("position_comparison.txt", "w").close()
        open("rotation_current.txt", "w").close()
        open("rotation_integrated.txt", "w").close()
        #if self.uav == 'firefly': 
        #    self.nrotors = 6
        #else: 
        #    self.nrotors = 4
        #self.pub = rospy.Publisher('/'+self.uav+'/command/motor_speed', Actuators, queue_size = 1, tcp_nodelay = True)
        try:            
            odom = rospy.Subscriber('/'+self.uav+'/odometry_sensor1/odometry', Odometry, self.ic_callback, queue_size = 100, tcp_nodelay = True) 
            ms = rospy.Subscriber('/'+self.uav+'/command/motor_speed', Actuators, self.integration_callback, queue_size = 100, tcp_nodelay = True)
            #odom = message_filters.Subscriber('/'+self.uav+'/odometry_sensor1/odometry', Odometry, tcp_nodelay = True) 
            #ms = message_filters.Subscriber('/'+self.uav+'/command/motor_speed', Actuators, tcp_nodelay = True)
            #ts = message_filters.TimeSynchronizer([odom, ms], 100)
            #ts.registerCallback(self.callback)
        except:
            print('problem subscribing to motor speeds')
    def ic_callback(self, odom): 
        rospy.loginfo('counter:%d', self.initial_condition_counter)
        if self.initial_condition_counter == 0: 
            self.x = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
            q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
            self.R = q.rotation_matrix
            _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
            self.v = np.dot(self.R, _V)
            self.omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
            self.omega1 = self.omega
            self.currentomega = self.omega
            self.currentv = self.v
            self.Rc = self.R
            self.currentx = self.x
            
        else: 

            self.currentomega  = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
            q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
            self.Rc = q.rotation_matrix
            _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
            self.currentv = np.dot(self.Rc, _V)
            self.currentx = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
            if self.initial_condition_counter < 25:
                self.omega = self.currentomega
                self.v = self.currentv
            
        self.initial_condition_counter += 1
   
    def integration_callback(self, ms): 
        #print 'integration_callback'

        w = ar([[i**2] for i in ms.angular_velocities])        
        c1 = tp(ar([[sin(pi/6)*self.d*self.tau_t, -cos(pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
        c2 = tp(ar([[sin(pi/2)*self.d*self.tau_t, -cos(pi/2)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c3 = tp(ar([[sin(5*pi/6)*self.d*self.tau_t, -cos(5*pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
        c4 = tp(ar([[sin(7*pi/6)*self.d*self.tau_t, -cos(7*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c5 = tp(ar([[sin(3*pi/2)*self.d*self.tau_t, -cos(3*pi/2)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
        c6 = tp(ar([[sin(11*pi/6)*self.d*self.tau_t, -cos(11*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        C = np.column_stack((c1,c2,c3,c4,c5,c6))
        T = np.dot(C, w)

        #if np.linalg.norm(self.currentomega) > 0.00001:
        
        if self.initial_condition_counter < 25:
            rospy.loginfo('initial period')
           
        else:
            
            
            #T = np.array([[ms.Moment_x], [ms.Moment_y], [ms.Moment_z], [ms.Total_thrust]])
            f1= open('forces_using_motor_speed.txt', 'a')
            f1.write("%s, %s, %s, %s, %s\n" % (ms.header.stamp, T[0][0], T[1][0], T[2][0], T[3][0]))
            force = T[3]; moments = T[0:3]
            
            f2= open('angular_velocity_comparison.txt', 'a')
            f2.write("%s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, self.currentomega[0][0], self.omega[0][0], self.currentomega[1][0],\
            self.omega[1][0], self.currentomega[2][0], self.omega[2][0]))
            
    
    
            f11 = open('rotation_current.txt', 'a') 
            f11.write("%s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (self.Rc[0][0], self.Rc[0][1], self.Rc[0][2], self.Rc[1][0],\
            self.Rc[1][1], self.Rc[1][2], self.Rc[2][0], self.Rc[2][1], self.Rc[2][2]))
    
            f12 = open('rotation_integrated.txt', 'a') 
            f12.write("%s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (self.R[0][0], self.R[0][1], self.R[0][2], self.R[1][0],\
            self.R[1][1], self.R[1][2], self.R[2][0], self.R[2][1], self.R[2][2]))    
    
            f4= open('linear_velocity_comparison.txt', 'a')
            f4.write("%s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, self.currentv[0][0], self.v[0][0], self.currentv[1][0],\
            self.v[1][0], self.currentv[2][0], self.v[2][0]))        
            f5= open('position_comparison.txt', 'a')
            f5.write("%s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, self.currentx[0][0], self.x[0][0], self.currentx[1][0],\
            self.x[1][0], self.currentx[2][0], self.x[2][0]))  
    
    
            #f6= open('omega_from_two_methods.txt', 'a')
            #f6.write("%s, %s, %s, %s, %s, %s\n" % (self.omega1[0][0], self.omega1[1][0], self.omega1[2][0],\
            #self.omega[0][0], self.omega[1][0], self.omega[2][0])) 
            
            #w1 = self.omega[0][0]; w2 = self.omega[1][0]; w3 = self.omega[2][0] 
            #J1 = self.J[0][0]; J2 = self.J[1][1]; J3 = self.J[2][2]        
            #w1 = w1 + self.dt * (moments[0][0]-w2*w3*(J3-J2))/J1
            #w2 = w2 + self.dt * (moments[1][0]-w1*w3*(J1-J3))/J2a
            #w3 = w3 + self.dt * (moments[2][0]-w2*w1*(J2-J1))/J3        
            #self.omega1 = ar([[w1], [w2], [w3]])
    
           
            
            a = tp(np.cross(self.omega.T, tp(np.dot(self.J,self.omega))))
            intermediate = moments-tp(np.cross(self.omega.T, tp(np.dot(self.J,self.omega))))        
            self.omega = self.omega + self.dt* np.dot(self.Jinv, intermediate)        
            b = np.dot(np.linalg.inv(self.J), intermediate)
            
            
            
            f3= open('components_in_moments.txt', 'a')
            f3.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, moments[0][0], moments[1][0], moments[2][0], a[0][0], a[1][0], a[2][0], b[0][0], b[1][0], b[2][0]))  
            
    
            omega_hat = ar([[0, -self.omega[2][0], self.omega[1][0]], [self.omega[2][0], 0, -self.omega[0][0]], [-self.omega[1][0], self.omega[0][0], 0]])
            
            self.x = self.x + self.dt*self.v
            self.v = self.v + self.dt *(force[0]*self.Rc[:,2][np.newaxis].T/self.m - self.g*self.e[:,2][np.newaxis].T)
            self.R = self.R + self.dt * np.dot(self.R, omega_hat)
        
    def callback(self, odom, ms): 
        if self.initial_condition_counter == 0: 
            self.x = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
            q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
            self.R = q.rotation_matrix
            _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
            self.v = np.dot(self.R, _V)
            self.omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
            self.omega1 = self.omega
            self.currentomega = self.omega
            self.currentv = self.v
            self.Rc = self.R
            self.currentx = self.x
        else: 

            self.currentomega  = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
            q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
            self.Rc = q.rotation_matrix
            _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
            self.currentv = np.dot(self.Rc, _V)
            self.currentx = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
            if self.initial_condition_counter < 2:
                self.omega = self.currentomega
                self.v = self.currentv
            
        self.initial_condition_counter += 1
        
        w = ar([[i**2] for i in ms.angular_velocities])        
        c1 = tp(ar([[sin(pi/6)*self.d*self.tau_t, -cos(pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
        c2 = tp(ar([[sin(pi/2)*self.d*self.tau_t, -cos(pi/2)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c3 = tp(ar([[sin(5*pi/6)*self.d*self.tau_t, -cos(5*pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
        c4 = tp(ar([[sin(7*pi/6)*self.d*self.tau_t, -cos(7*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c5 = tp(ar([[sin(3*pi/2)*self.d*self.tau_t, -cos(3*pi/2)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
        c6 = tp(ar([[sin(11*pi/6)*self.d*self.tau_t, -cos(11*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        C = np.column_stack((c1,c2,c3,c4,c5,c6))
        T = np.dot(C, w)

        #if np.linalg.norm(self.currentomega) > 0.00001:
        
        #if self.initial_condition_counter < 2:
        #    rospy.loginfo('initial period')
            
        #else:
            
            
        #T = np.array([[ms.Moment_x], [ms.Moment_y], [ms.Moment_z], [ms.Total_thrust]])
        f1= open('forces_using_motor_speed.txt', 'a')
        f1.write("%s, %s, %s, %s, %s\n" % (ms.header.stamp, T[0][0], T[1][0], T[2][0], T[3][0]))
        force = T[3]; moments = T[0:3]
        
        f2= open('angular_velocity_comparison.txt', 'a')
        f2.write("%s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, self.currentomega[0][0], self.omega[0][0], self.currentomega[1][0],\
        self.omega[1][0], self.currentomega[2][0], self.omega[2][0]))
        


        f11 = open('rotation_current.txt', 'a') 
        f11.write("%s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (self.Rc[0][0], self.Rc[0][1], self.Rc[0][2], self.Rc[1][0],\
        self.Rc[1][1], self.Rc[1][2], self.Rc[2][0], self.Rc[2][1], self.Rc[2][2]))

        f12 = open('rotation_integrated.txt', 'a') 
        f12.write("%s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (self.R[0][0], self.R[0][1], self.R[0][2], self.R[1][0],\
        self.R[1][1], self.R[1][2], self.R[2][0], self.R[2][1], self.R[2][2]))    

        f4= open('linear_velocity_comparison.txt', 'a')
        f4.write("%s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, self.currentv[0][0], self.v[0][0], self.currentv[1][0],\
        self.v[1][0], self.currentv[2][0], self.v[2][0]))        
        f5= open('position_comparison.txt', 'a')
        f5.write("%s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, self.currentx[0][0], self.x[0][0], self.currentx[1][0],\
        self.x[1][0], self.currentx[2][0], self.x[2][0]))  


        #f6= open('omega_from_two_methods.txt', 'a')
        #f6.write("%s, %s, %s, %s, %s, %s\n" % (self.omega1[0][0], self.omega1[1][0], self.omega1[2][0],\
        #self.omega[0][0], self.omega[1][0], self.omega[2][0])) 
        
        #w1 = self.omega[0][0]; w2 = self.omega[1][0]; w3 = self.omega[2][0] 
        #J1 = self.J[0][0]; J2 = self.J[1][1]; J3 = self.J[2][2]        
        #w1 = w1 + self.dt * (moments[0][0]-w2*w3*(J3-J2))/J1
        #w2 = w2 + self.dt * (moments[1][0]-w1*w3*(J1-J3))/J2a
        #w3 = w3 + self.dt * (moments[2][0]-w2*w1*(J2-J1))/J3        
        #self.omega1 = ar([[w1], [w2], [w3]])

       
        
        a = tp(np.cross(self.omega.T, tp(np.dot(self.J,self.omega))))
        intermediate = moments-tp(np.cross(self.omega.T, tp(np.dot(self.J,self.omega))))        
        self.omega = self.omega + self.dt* np.dot(self.Jinv, intermediate)        
        b = np.dot(np.linalg.inv(self.J), intermediate)
        
        
        
        f3= open('components_in_moments.txt', 'a')
        f3.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (ms.header.stamp, moments[0][0], moments[1][0], moments[2][0], a[0][0], a[1][0], a[2][0], b[0][0], b[1][0], b[2][0]))  
        

        omega_hat = ar([[0, -self.omega[2][0], self.omega[1][0]], [self.omega[2][0], 0, -self.omega[0][0]], [-self.omega[1][0], self.omega[0][0], 0]])
        
        self.x = self.x + self.dt*self.v
        self.v = self.v + self.dt *(force[0]*self.Rc[:,2][np.newaxis].T/self.m - self.g*self.e[:,2][np.newaxis].T)
        self.R = self.R + self.dt * np.dot(self.R, omega_hat)

if __name__ == '__main__':
    name = 'firefly'#rospy.get_param('uav_name')
    rospy.init_node('integrating_mav_dynamics', anonymous=False)
    r = rospy.Rate(200)
    p = parameters(name)
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]
    c = integrating_mav_dynamics(name, parameter_list)
    try: 
        while not rospy.is_shutdown(): 
            #c = integrating_mav_dynamics(name, parameter_list)
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass

