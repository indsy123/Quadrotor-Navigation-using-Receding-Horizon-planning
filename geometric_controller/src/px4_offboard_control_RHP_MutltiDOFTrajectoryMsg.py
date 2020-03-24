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
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class px4_offboard_control(object):
    def __init__(self, name, number, mass, kx, kv, norm_thrust, rate_gain_rp, rate_gain_yaw, RN):
        """initializes the variables"""
        self.path = '/home/pelican/data_collection/rhp_logs/'
        open(self.path + "position_trajectory_RN{}.txt".format(RN), "w").close()
        open(self.path +"velocity_trajectory_RN{}.txt".format(RN), "w").close()
        open(self.path + 'commands_generated_RN{}.txt'.format(RN), "w").close()
        self.RN = RN
        no = str(number)
        self.counter = 0
        self.g = 9.86
        self.m = mass
        self.kx = np.asarray(kx)[np.newaxis].T
        self.kv = np.asarray(kv)[np.newaxis].T
        self.e = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.norm_thrust_constant = norm_thrust
        self.factor_rp = 2.0/rate_gain_rp
        self.factor_yaw = 2.0/rate_gain_yaw
        self.controller = 0
        self.despos = []; self.desvel = []; self.desacc = []; self.desdirection = []; self.trajectory_time = []
        self.pub = rospy.Publisher('/'+name+no+"/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 100, tcp_nodelay = True)

        try: 
            self.arming_service = rospy.ServiceProxy('/'+name+no+"/mavros/cmd/arming", CommandBool)
            self.arming_service.call(True)
            rospy.Subscriber('/'+name+no+'/modifiedodom', Odometry, self.odom_callback, tcp_nodelay = True)
            rospy.Subscriber('/'+name+no+'/polynomial_trajectory', MultiDOFJointTrajectory, self.traj_callback, tcp_nodelay = True)
        except: 
            print "Probem subscribing to mocap topic"

    def traj_callback(self, traj): 
        """stores the trajectory"""  
        for i in range(len(traj.points)): 
            posx = traj.points[i].transforms[0].translation.x
            posy = traj.points[i].transforms[0].translation.y
            posz = traj.points[i].transforms[0].translation.z
            
            velx = traj.points[i].velocities[0].linear.x
            vely = traj.points[i].velocities[0].linear.y
            velz = traj.points[i].velocities[0].linear.z
            
            accx = traj.points[i].accelerations[0].linear.x
            accy = traj.points[i].accelerations[0].linear.y
            accz = traj.points[i].accelerations[0].linear.z
            
            dirx = traj.points[i].accelerations[0].angular.x
            diry = traj.points[i].accelerations[0].angular.y
            dirz = traj.points[i].accelerations[0].angular.z
            tt = traj.points[i].time_from_start.to_sec()

            self.despos.append(ar([[posx], [posy], [posz]]))
            self.desvel.append(ar([[velx], [vely], [velz]]))
            self.desacc.append(ar([[accx], [accy], [accz]]))
            self.desdirection.append(ar([[dirx], [diry], [dirz]])) # a dummy direction for now 
            self.trajectory_time.append(tt)
            
            self.controller = 0 # position controller 
            f4 = open(self.path+'subscribed_trajectory_RN{}.txt'.format(self.RN),'a')
            f4.write("%s, %s, %s, %s\n" % (tt, posx, posy, posz))
            if len(self.trajectory_time)>500:
                self.trajectory_time.pop(0); self.despos.pop(0); self.desvel.pop(0); self.desacc.pop(0); self.desdirection.pop(0)


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
        """send commands to px4"""
        current_time = odom.header.stamp.to_sec()
        self.ct = current_time
        if self.counter == 0: 
            self.initial_odom_time = current_time


        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]])
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])

        
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME.
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        #Omega = np.dot(R.T, Omega) # this is needed because "vicon" package from Upenn publishes spatial angular velocities

        try: 
            index, value = min(enumerate(self.trajectory_time), key=lambda x: abs(x[1]-current_time))
            Xd = self.despos[index]; Vd = self.desvel[index]; ad = self.desacc[index]; b1d = self.desdirection[index]
            #print index, Xd, Vd, ad
        except: 
            rospy.loginfo('Trajectory has not yet been received.')
        if self.controller == 1: # velocity controller
            ex = ar([[0],[0],[0]])
        else: 
            ex = X-Xd
            
        ev = V-Vd
    
 
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix`

        qc = self.rotmat2quat(Rc)
        omega_des = q.inverse*qc
            
    
        thrust = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))/self.norm_thrust_constant # normalized thrust 
            
        msg = AttitudeTarget()
        msg.header.stamp = odom.header.stamp
        msg.type_mask = 128 
            
        msg.body_rate.x = self.factor_rp*np.sign(omega_des[0])*omega_des[1]
        msg.body_rate.y = self.factor_rp*np.sign(omega_des[0])*omega_des[2]
        msg.body_rate.z = self.factor_yaw*np.sign(omega_des[0])*omega_des[3]
    
        msg.thrust = min(1.0, thrust[0][0])
        #print 'thrust:', thrust*self.norm_thrust_constant, 'thrust_factor:', min(1.0, thrust[0][0])
        #print 'omega_des',msg.body_rate.x, msg.body_rate.y, msg.body_rate.z
        #print 'zheight', Xd[2][0]

        f1 = open(self.path+'position_trajectory_RN{}.txt'.format(self.RN), 'a')
        f1.write("%s, %s, %s, %s, %s, %s\n" % (Xd[0][0], Xd[1][0], Xd[2][0], X[0][0], X[1][0], X[2][0]))
        f2 = open(self.path+'velocity_trajectory_RN{}.txt'.format(self.RN), 'a')
        f2.write("%s, %s, %s, %s, %s, %s\n" % (Vd[0][0], Vd[1][0], Vd[2][0], V[0][0], V[1][0], V[2][0]))
        f3 = open(self.path + 'commands_generated_RN{}.txt'.format(self.RN), 'a')
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
    rate_gain_rp = rospy.get_param('rate_gain_rp')
    rate_gain_yaw = rospy.get_param('rate_gain_yaw')
    RN = rospy.get_param('run_number')
    rospy.init_node('sendcommandtopx4', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    c = px4_offboard_control(name, number, mass, kx, kv, norm_thrust, rate_gain_rp, rate_gain_yaw, RN)
               
    try: 
        while not rospy.is_shutdown(): 
         rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
