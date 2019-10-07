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
from geometric_controller.msg import Desired_Trajectory, control_inputs, modifiedodometry
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
from numpy import sin as sin
from numpy import cos as cos
from numpy import pi as pi 
import scipy, bisect
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators
from geometric_controller.msg import PolynomialCoefficients
from trajectory_msgs.msg import MultiDOFJointTrajectory

class velocity_controller(object): 
    def __init__(self, name_of_uav, number, freq, parameters): 
        self.uav = name_of_uav
        self.no = str(number)
        self.m = parameters[0]
        self.J = parameters[1]
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
        self.controller = 0 # default to position controller
        self.traj_counter = 0
        self.initial_odom_time  = 0
        self.odom_time = 0
        self.traj_start_time = 0
        self.traj_end_time = []
        self.number_of_segments = 0
        self.p1 = []; self.p2 = []; self.p3 = []
        open("trajectory_generated_subscribed.txt", "w").close()
        open("trajectory_subscribed.txt", "w").close()
        open("trajectory_followed.txt", "w").close()
        open("motor_speeds.txt", "w").close()
        self.kR_normalized = np.dot(np.linalg.inv(self.J), self.kR)
        self.kOmega_normalized = np.dot(np.linalg.inv(self.J), self.kOmega)
        self.time_points = []; self.time = time.time()
        self.smooth_vx = []; self.smooth_vy = []; self.smooth_vz = []
        self.dt = 1.0/freq # time step based on 150Hz frequency
        self.ei = 0
        self.despos = []; self.desvel = []; self.desacc = []; self.desdirection = []; self.trajectory_time = []
        self.dpos = ar([[-5.0], [4.0], [2.0]]); self.dvel = ar([[0], [0], [0]]); self.dacc = ar([[0], [0], [0]])
        self.ddir = ar([[1], [0], [0]])
        #self.despos = ar([[-6.5],[-4.0],[0.07]]); self.desvel = ar([[0],[0],[0]]); self.desacc = ar([[0],[0],[0]])
        #elf.desdirection = ar([[1],[0],[0]]) 
        self.controller = 0 # default to position controller 
        self.pub = rospy.Publisher('/firefly'+self.no +'/command/motor_speed', Actuators, queue_size = 1, tcp_nodelay = True)
        self.start_time = time.time()
        rospy.Subscriber('/firefly'+self.no +'/odometry_sensor1/odometry', Odometry, self.odom_callback, queue_size = 1, tcp_nodelay = True)
        #rospy.Subscriber('/firefly'+self.no +'/polynomial_trajectory', MultiDOFJointTrajectory, self.traj_callback, queue_size = 1, tcp_nodelay = True)
        rospy.Subscriber('/firefly1/polynomial_coefficients', PolynomialCoefficients, self.traj_callback, tcp_nodelay = True)


    def traj_callback(self, traj): 
        """stores the trajectory"""  
        
        pp1 = traj.poly_x; pp2 = traj.poly_y; pp3 = traj.poly_z
        N = traj.polynomial_order + 1
        
        self.p1 = [list(pp1[i:i + N]) for i in range(0, len(pp1), N)]
        self.p2 = [list(pp2[i:i + N]) for i in range(0, len(pp2), N)]
        self.p3 = [list(pp3[i:i + N]) for i in range(0, len(pp3), N)]
        [i.reverse() for i in self.p1]
        [i.reverse() for i in self.p2]
        [i.reverse() for i in self.p3]
        
        self.ddir = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])
        self.traj_start_time = traj.planning_start_time
        self.traj_end_time = traj.segment_end_times
        self.number_of_segments = traj.number_of_segments
        
    def odom_callback(self, odom): 
        """does the control calculations"""
        self.odom_time = odom.header.stamp.to_sec()

        t = float(self.odom_time - self.traj_start_time)
        try: 
            if t < self.traj_end_time[self.number_of_segments]: 
                index = bisect.bisect(self.traj_end_time, t)-1
            else: 
                index = bisect.bisect(self.traj_end_time, t)-2
            #fff = open('traj_time_in_odom.txt', 'a')
            #fff.write('%s, %s, %s, %s, %s\n' %(index, self.odom_time, self.traj_start_time, t, self.traj_end_time[self.number_of_segments]))
            if index == -1: 
                pass
            else:        
    
                xx = np.poly1d(self.p1[index])
                vx = np.polyder(xx, 1); aax = np.polyder(xx, 2)
                jx = np.polyder(xx, 3); sx = np.polyder(xx, 4)
    
                yy = np.poly1d(self.p2[index])
                vy = np.polyder(yy, 1); aay = np.polyder(yy, 2)
                jy = np.polyder(yy, 3); sy = np.polyder(yy, 4)
                
                zz = np.poly1d(self.p3[index])
                vz = np.polyder(zz, 1); aaz = np.polyder(zz, 2)
                jz = np.polyder(zz, 3); sz = np.polyder(zz, 4)
                if t < self.traj_end_time[self.number_of_segments]:
                    self.pdes = ar([[xx(t)], [yy(t)], [zz(t)]]) 
                    self.vdes = ar([[vx(t)], [vy(t)], [vz(t)]])
                    self.ades = ar([[aax(t)], [aay(t)], [aaz(t)]])
                    self.jdes = ar([[jx(t)], [jy(t)], [jz(t)]])
                    self.ddes = self.ddir
                else: 
                    self.pdes = ar([[xx(self.traj_end_time[-1])], [yy(self.traj_end_time[-1])], [zz(self.traj_end_time[-1])]]) 
                    self.vdes = ar([[vx(self.traj_end_time[-1])], [vy(self.traj_end_time[-1])], [vz(self.traj_end_time[-1])]])
                    self.ades = ar([[aax(self.traj_end_time[-1])], [aay(self.traj_end_time[-1])], [aaz(self.traj_end_time[-1])]])
                    self.jdes = ar([[jx(self.traj_end_time[-1])], [jy(self.traj_end_time[-1])], [jz(self.traj_end_time[-1])]])
                    self.ddes = self.ddir
                
                current_time = time.time()-self.start_time
                if self.counter == 0: 
                    self.initial_odom_time = current_time

            
            X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
            q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
            R = q.rotation_matrix
            # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
            _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
            #acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z]])
            #V1 = _V#np.dot(R, _V);
            V1 = np.dot(R, _V)
            
            # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
            Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
            #Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities
    
         
            # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction
            #index, value = min(enumerate(self.trajectory_time), key=lambda x: abs(x[1]-current_time))
            #Xd = self.despos[index]; Vd = self.desvel[index]; ad = self.desacc[index]; b1d = self.desdirection[index] 
    
            Xd = self.pdes; Vd = self.vdes; ad = self.ades; b1d = self.ddes
            ad_dot = self.jdes
            if self.controller == 1: # velocity controller
                ex = ar([[0],[0],[0]])
            else: 
                ex = X-Xd
            
            ev = V1-Vd
            _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
            b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
            b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
            b1c = tp(np.cross(b2c.T, b3c.T))
            Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix
            
            ff = open('trajectory_subscribed.txt', 'a')
            ff.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (index, current_time, Xd[0][0], Xd[1][0], Xd[2][0], Vd[0][0], Vd[1][0], Vd[2][0], ad[0][0], ad[1][0], ad[2][0]))
            ff = open('trajectory_followed.txt', 'a')
            ff.write("%s, %s, %s, %s, %s, %s\n" % (X[0][0], X[1][0], X[2][0], V1[0][0], V1[1][0], V1[2][0]))
            """       
            phi = np.arctan2(b2c[1][0], b2c[0][0])
            
    
            if phi < 0: 
                phi = phi + 2 * np.pi
    
            error  = 0.5 * np.trace(self.e-np.dot(Rc.T, R))
    
            
            if self.counter != 0:
                phi_dot = (phi-self.phi_)/self.dt; self.phi_ = phi
            else: 
    
                phi_dot = 0; self.phi_ = phi
    
            Omega_c = ar([[0], [0], [phi_dot]])
            
            """
            if self.counter == 0:
                acc = tp(np.zeros((1,3))); self.previous_V = V1
                b1d_dot = tp(np.zeros((1,3))); self.previous_b1d = b1d
            else: 
                acc = (V1-self.previous_V)/self.dt; self.previous_V = V1
                b1d_dot = (b1d-self.previous_b1d)/self.dt; self.previous_b1d = b1d
                
            # calculate derivative of rotation and find desired angular velocity        
            A = (mult(self.kx, ex) + mult(self.kv, ev))/self.m - self.g*self.e[:,2][np.newaxis].T - ad
            ex_dot = ev; ev_dot = acc-ad
            A_dot = (mult(self.kx, ex_dot) + mult(self.kv, ev_dot))/self.m - ad_dot # calculate desired direction 
            b3c_dot = -A_dot/np.linalg.norm(A) + (np.dot(A.T,A_dot)/np.linalg.norm(A)**3)*A
            
            C = tp(np.cross(b3c.T,b1d.T))
            C_dot = tp(np.cross(b3c_dot.T, b1d.T) + np.cross(b3c.T, b1d_dot.T))
            b2c_dot = C_dot/np.linalg.norm(C) - (np.dot(C.T,C_dot)/np.linalg.norm(C)**3)*C
            b1c_dot = tp(np.cross(b2c_dot.T, b3c.T) + np.cross(b2c.T, b3c_dot.T))
            
            Rc_dot = np.column_stack((b1c_dot, b2c_dot, b3c_dot)) # desired rotation matrix derivative
            
            #Omega_c_hat = np.dot(tp(Rc),Rc_dot) # skew-symmetric desired angular velocity 
            Omega_c_hat = np.dot(Rc.T, Rc_dot)
            #Omega_c_hat = ar([[0,0,0], [0,0,0], [0, 0, 0]])
            Omega_c = ar([[-Omega_c_hat[1][2]], [Omega_c_hat[0][2]], [-Omega_c_hat[0][1]]])        
           
            
            
            eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix
    
            eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
            eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity
    
            self.f = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))
            self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))# - np.dot(self.J,Z)
    
            T = tp(ar([[self.M[0][0],self.M[1][0],self.M[2][0],self.f[0][0]]])) # dummy torque to offset visensor moment

    
            Msg = Actuators()
            if self.uav == 'hummingbird':
                c1 = tp(ar([[0, 2/(self.d*self.tau_t), 0, -2/(self.d*self.tau_t)]]))
                c2 = tp(ar([[-2/(self.d*self.tau_t), 0, 2/(self.d*self.tau_t), 0]]))
                c3 = tp(ar([[1/(self.tau_t*self.m), -1/(self.tau_t*self.m), 1/(self.tau_t*self.m), -1/(self.tau_t*self.m)]]))
                c4 = tp(ar([[1/self.tau_t, 1/self.tau_t, 1/self.tau_t, 1/self.tau_t]]))
                C = 0.25*np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2 
                w_square = np.dot(C,T)
                #w_initial = np.array([[self.w0_h**2], [self.w0_h**2], [self.w0_h**2], [self.w0_h**2]])
                w = np.sqrt(np.abs(w_square))
                w[w>800.0] = 800.0
                Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]
    
            elif self.uav== 'pelican': 
                c1 = tp(ar([[0, 2/(self.d*self.tau_t), 0, -2/(self.d*self.tau_t)]]))
                c2 = tp(ar([[-2/(self.d*self.tau_t), 0, 2/(self.d*self.tau_t), 0]]))
                c3 = tp(ar([[1/(self.tau_t*self.m), -1/(self.tau_t*self.m), 1/(self.tau_t*self.m), -1/(self.tau_t*self.m)]]))
                c4 = tp(ar([[1/self.tau_t, 1/self.tau_t, 1/self.tau_t, 1/self.tau_t]]))
                C = 0.25*np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2 
                w_square = np.dot(C,T)
                w = np.sqrt(np.abs(w_square))
                w[w>800.0] = 800.0
                Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]
    
            else: 
                c1 = tp(ar([[sin(pi/6)*self.d*self.tau_t, -cos(pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
                c2 = tp(ar([[sin(pi/2)*self.d*self.tau_t, -cos(pi/2)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
                c3 = tp(ar([[sin(5*pi/6)*self.d*self.tau_t, -cos(5*pi/6)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
                c4 = tp(ar([[sin(7*pi/6)*self.d*self.tau_t, -cos(7*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
                c5 = tp(ar([[sin(3*pi/2)*self.d*self.tau_t, -cos(3*pi/2)*self.d*self.tau_t, -self.tau_t*self.tau_m, self.tau_t]]))
                c6 = tp(ar([[sin(11*pi/6)*self.d*self.tau_t, -cos(11*pi/6)*self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
                C = np.column_stack((c1,c2,c3,c4,c5,c6)) # solving linear eq T = Cw^2 to get w^2
                w_square = np.dot(np.linalg.pinv(C),T)
                w = np.sqrt(np.abs(w_square))
                #w[w>900.0] = 900.0
                Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0], w[4][0], w[5][0]]
            
            ff = open('motor_speeds.txt', 'a')
            ff.write("%s, %s, %s, %s, %s, %s\n" % (w[0][0], w[1][0], w[2][0], w[3][0], w[4][0], w[5][0]))
            self.pub.publish(Msg)
            self.counter+=1
        except: 
            print 'No trajectory is being published..check trajectory generation algorithm..'

if __name__ == '__main__':
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    freq = rospy.get_param('publish_frequency')
    rospy.init_node('GeometricController', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    p = parameters(name)
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]
    c = velocity_controller(name, number, freq, parameter_list)
    try: 
        while not rospy.is_shutdown(): 
            
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass

