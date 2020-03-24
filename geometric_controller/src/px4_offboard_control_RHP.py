#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 23 10:46:20 2018

@author: indrajeet
Controller for receding horizon planning in camera field of view. 
"""

import numpy as np 
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
import bisect
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
import message_filters
import time
from geometric_controller.msg import PolynomialCoefficients
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
from mavros_msgs.msg import ActuatorControl, Thrust, AttitudeTarget


class px4_offboard_control(object):
    def __init__(self, name, number, mass, kx, kv, norm_thrust, rate_gain):
        """initializes the variables"""
        self.path = '/home/pelican/data_collection/'
        open(self.path + "position_trajectory.txt", "w").close()
        open(self.path +"velocity_trajectory.txt", "w").close()
        open(self.path + 'commands.txt', "w").close()
        #self.uav = name_of_uav
        no = str(number)
        self.counter = 0
        self.g = 9.86; self.m = mass; #self.dt = 1.0/100
        self.kx = np.asarray(kx)[np.newaxis].T
        self.kv = np.asarray(kv)[np.newaxis].T
        self.e = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.norm_thrust_constant = norm_thrust
        self.factor = 2.0/rate_gain
        self.p1 = []; self.p2 = []; self.p3 = []
        self.controller = 0
        self.traj_end_time = []
        self.traj_start_time = 0
        self.start_time = time.time()
        self.pub = rospy.Publisher('/'+name+no+"/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 100, tcp_nodelay = True)

        try: 
            self.arming_service = rospy.ServiceProxy('/'+name+no+"/mavros/cmd/arming", CommandBool)
            self.arming_service.call(True)
            rospy.Subscriber('/'+name+no+'/modifiedodom', Odometry, self.odom_callback, tcp_nodelay = True)
            rospy.Subscriber('/'+name+no+'/polynomial_coefficients', PolynomialCoefficients, self.traj_callback, tcp_nodelay = True)
            
        except: 
            print "Probem subscribing to mocap topic"

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
        
        self.ddes = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])
        self.traj_start_time = float(traj.planning_start_time)
        self.traj_end_time = traj.segment_end_times
        self.number_of_segments = traj.number_of_segments
        self.mode = traj.trajectory_mode
        self.execution_time_horizon = float(traj.execution_time_horizon)


    def rotmat2quat(self, RR): 
        """function to convert a given rotation matrix to a quaternion """ 
        trace = np.trace(RR)
        if trace >= 0:
            S = 2.0 * np.sqrt(trace + 1)
            qw = 0.25 * S
            qx = (RR[2][1] - RR[1][2]) / S
            qy = (RR[0][2] - RR[2][0]) / S
            qz = (RR[1][0] - RR[0][1]) / S
        elif RR[0][0] > RR[1][1] and RR[0][0] > RR[2][2]: 
            S = 2.0 * np.sqrt(1 + RR[0][0] - RR[1][1] - RR[2][2])
            qw = (RR[2][1] - RR[1][2]) / S
            qx = 0.25 * S
            qy = (RR[1][0] + RR[0][1]) / S
            qz = (RR[0][2] + RR[2][0]) / S
        elif RR[1][1] > RR[2][2]:
            S = 2.0 * np.sqrt(1 + RR[1][1] - RR[0][0] - RR[2][2])
            qw = (RR[0][2] - RR[2][0]) / S
            qx = (RR[1][0] + RR[0][1]) / S
            qy = 0.25 * S
            qz = (RR[2][1] + RR[1][2]) / S
        else:
            S = 2.0 * np.sqrt(1 + RR[2][2] - RR[0][0] - RR[1][1])            
            qw = (RR[1][0] - RR[0][1]) / S
            qx = (RR[0][2] + RR[2][0]) / S            
            qy = (RR[2][1] + RR[1][2]) / S   
            qz = 0.25 * S
 
        return Quaternion(qw, qx, qy, qz)
        
    def odom_callback(self, odom):
        """send command to px4"""
	self.odom_time = odom.header.stamp.to_sec()
        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])

        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        #Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities

            
            
        t = float(self.odom_time - self.traj_start_time)

        if t <= self.traj_end_time[self.number_of_segments]: 
            index = bisect.bisect(self.traj_end_time, t)-1
        else: 
            index = bisect.bisect(self.traj_end_time, t)-2


        if index == -1: 
            pass
        else:        

            xx = np.poly1d(self.p1[index])
            vx = np.polyder(xx, 1); aax = np.polyder(xx, 2)
            jx = np.polyder(xx, 3)

            yy = np.poly1d(self.p2[index])
            vy = np.polyder(yy, 1); aay = np.polyder(yy, 2)
            jy = np.polyder(yy, 3)
            
            zz = np.poly1d(self.p3[index])
            vz = np.polyder(zz, 1); aaz = np.polyder(zz, 2)
            jz = np.polyder(zz, 3)


            if self.mode == 'hover' or self.mode == 'land': 
                if t <= self.traj_end_time[self.number_of_segments]:
                    self.pdes = ar([[xx(t)], [yy(t)], [zz(t)]])
                    self.vdes = ar([[vx(t)], [vy(t)], [vz(t)]])
                    self.ades = ar([[aax(t)], [aay(t)], [aaz(t)]])
                    self.jdes = ar([[jx(t)], [jy(t)], [jz(t)]])
                    #self.ddes = self.ddir
                else: 
                    self.pdes = ar([[xx(self.traj_end_time[-1])], [yy(self.traj_end_time[-1])], [zz(self.traj_end_time[-1])]])
                    self.vdes = ar([[vx(self.traj_end_time[-1])], [vy(self.traj_end_time[-1])], [vz(self.traj_end_time[-1])]])
                    self.ades = ar([[aax(self.traj_end_time[-1])], [aay(self.traj_end_time[-1])], [aaz(self.traj_end_time[-1])]])
                    self.jdes = ar([[jx(self.traj_end_time[-1])], [jy(self.traj_end_time[-1])], [jz(self.traj_end_time[-1])]])
                    #self.ddes = self.ddir
            else: 
                if t<= self.execution_time_horizon: 
                    self.pdes = ar([[xx(t)], [yy(t)], [zz(t)]])
                    self.vdes = ar([[vx(t)], [vy(t)], [vz(t)]])
                    self.ades = ar([[aax(t)], [aay(t)], [aaz(t)]])
                    self.jdes = ar([[jx(t)], [jy(t)], [jz(t)]])
                    #self.ddes = self.ddir 
                else: 
                    ttt = self.execution_time_horizon
                    self.pdes = ar([[xx(ttt)], [yy(ttt)], [zz(ttt)]])
                    self.vdes = ar([[vx(ttt)], [vy(ttt)], [vz(ttt)]])
                    self.ades = ar([[aax(ttt)], [aay(ttt)], [aaz(ttt)]])
                    self.jdes = ar([[jx(ttt)], [jy(ttt)], [jz(ttt)]])
                    #self.ddes = self.ddir   

            current_time = time.time()-self.start_time
            if self.counter == 0:  
                self.initial_odom_time = current_time


            Xd = self.pdes; Vd = self.vdes; ad = self.ades; b1d = self.ddes
            b1d_dot = ar([[0],[0],[0]]); ad_dot = self.jdes
            if self.controller == 1: # velocity controller
                ex = ar([[0],[0],[0]])
            else: 
                ex = X-Xd
            
            ev = V-Vd
    
            #_b3c = -(mult(self.kx, ex) + mult(self.kv, ev)+ mult(self.ki, ei))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction 
            _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
            b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
            b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
            b1c = tp(np.cross(b2c.T, b3c.T))
            Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix`

            qc = self.rotmat2quat(Rc)
            
            
            omega_des = q.inverse*qc
            
    
            self.f = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))/self.norm_thrust_constant # normalized thrust 
            
            #self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega))))
            msg = AttitudeTarget()
            msg.header.stamp = odom.header.stamp
            msg.type_mask = 128 
            
            msg.body_rate.x = self.factor*np.sign(omega_des[0])*omega_des[1]
            msg.body_rate.y = self.factor*np.sign(omega_des[0])*omega_des[2]
            msg.body_rate.z = self.factor*np.sign(omega_des[0])*omega_des[3]
    
            msg.thrust = min(1.0, self.f[0][0])
            print 'thrust:', self.f*self.norm_thrust_constant, 'thrust_factor:', min(1.0, self.f[0][0])
            print 'omega_des',msg.body_rate.x, msg.body_rate.y, msg.body_rate.z
            print 'zheight', Xd[2][0]

            f1 = open(self.path+'position_trajectory.txt', 'a')
            f1.write("%s, %s, %s, %s, %s, %s\n" % (Xd[0][0], Xd[1][0], Xd[2][0], X[0][0], X[1][0], X[2][0]))
            f2 = open(self.path+'velocity_trajectory.txt', 'a')
            f2.write("%s, %s, %s, %s, %s, %s\n" % (Vd[0][0], Vd[1][0], Vd[2][0], V[0][0], V[1][0], V[2][0]))
            f3 = open(self.path + 'commands.txt', 'a')
            f3.write("%s, %s, %s, %s\n" % (msg.thrust, msg.body_rate.x, msg.body_rate.y, msg.body_rate.z)) 

            
            self.counter = self.counter+1
            self.pub.publish(msg)


if __name__ == '__main__':
    name = rospy.get_param('uav_name')
    number = rospy.get_param('number')
    mass = rospy.get_param('mass')
    kx = rospy.get_param('kx')
    kv = rospy.get_param('kv')
    norm_thrust = rospy.get_param('normalized_thrust_constant')
    rate_gain = rospy.get_param('rate_gain')
    rospy.init_node('sendcommandtopx4', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    c = px4_offboard_control(name, number, mass, kx, kv, norm_thrust, rate_gain)
                     
                      
    try: 
        while not rospy.is_shutdown(): 
         rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
