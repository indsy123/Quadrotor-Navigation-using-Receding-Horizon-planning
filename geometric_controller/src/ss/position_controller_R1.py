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
from isy_geometric_controller.msg import Desired_Trajectory, control_inputs
import message_filters
from numpy import array as ar # just to reduce typing 
from numpy import transpose as tp # just to reduce typing 
from numpy import multiply as mult # just to reduce typing 
#from tf import transformations
#from trajectory import tajectory 
from mav_msgs.msg import Actuators
#from scipy.interpolate import splrep, splev

class position_controller(object): 
    def __init__(self, name_of_uav,parameters,time_instance): 
        self.uav = name_of_uav
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
        self.kR_normalized = np.dot(np.linalg.inv(self.J), self.kR)
        self.kOmega_normalized = np.dot(np.linalg.inv(self.J), self.kOmega)
        self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size = 1)
        self.time = time_instance
        print self.time, "self.time"
        #self.pub = rospy.Publisher('/control_inputs', control_inputs, queue_size = 1)
        #self.ad_series = []
        #self.ad_dot_series = []
        #self.acc_series = []
        #self.timeseries = []
        
        try:
            odom = rospy.Subscriber('/pelican/odometry_sensor1/odometry', Odometry, self.callback,queue_size = 1)
            #traj = message_filters.Subscriber('/desired_trajectory', Desired_Trajectory)
            #accel = message_filters.Subscriber('/pelican/imu', Imu)
            #ts = message_filters.TimeSynchronizer([odom, accel, traj], 1)
            #ts.registerCallback(self.callback)
            #rospy.spin()
        except:
            print('problem subscribing to one of the topic above')
   
    def callback(self, odom): 
        a = time.time()
        
        print 'parameters',self.kR, self.kx, self.kv

        X = ar([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [odom.pose.pose.position.z]]);
        q = Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,\
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
        R = q.rotation_matrix
        # ensure that the linear velocity is in inertial frame, ETHZ code multiply linear vel to rotation matrix
        _V = ar([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z]])
        V = np.dot(R, _V)
        #acc = ar([[accel.linear_acceleration.x], [accel.linear_acceleration.y], [accel.linear_acceleration.z-9.8]])
        #acc = np.dot(R, _acc)
        #print acc
        # CROSSCHECK AND MAKE SURE THAT THE ANGULAR VELOCITY IS IN INERTIAL FRAME...HOW?
        Omega = ar([[odom.twist.twist.angular.x], [odom.twist.twist.angular.y], [odom.twist.twist.angular.z]])
        #Omega = np.dot(R, _Omega)
        """np.putmask(Omega, abs(Omega)<1e-9,0);"""
        Omega_hat = ar([[0,-Omega[2][0], Omega[1][0]], [Omega[2][0],0,-Omega[0][0]], [-Omega[1][0], Omega[0][0], 0]])
        """
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[traj.desired_position.x], [traj.desired_position.y], [traj.desired_position.z]])
 
        Vd = ar([[traj.desired_velocity.x], [traj.desired_velocity.y], [traj.desired_velocity.z]])
        ad = ar([[traj.desired_acceleration.x], [traj.desired_acceleration.y], [traj.desired_acceleration.z]])
        b1d = ar([[traj.desired_direction.x], [traj.desired_direction.y], [traj.desired_direction.z]])

        b1d_dot = ar([[traj.desired_direction_dot.x], [traj.desired_direction_dot.y], [traj.desired_direction_dot.z]])
        b1d_ddot = ar([[traj.desired_direction_ddot.x], [traj.desired_direction_ddot.y], [traj.desired_direction_ddot.z]])
        """
        print 'callback',self.time
        # Xd = desired position, Vd = desired velocity, ad = desired acceleration b1d: desired heading direction 
        Xd = ar([[0.1*self.time], [0.1*self.time], [1]])
 
        Vd = ar([[0.1], [0.1], [0]])
        ad = ar([[0], [0], [0]])
        b1d = ar([[1], [0], [0]])

        b1d_dot = ar([[0], [0], [0]])
        b1d_ddot = ar([[0], [0], [0]])
        
        
        correction = np.array([[0],[0],[0.1]])
        ex = X-Xd-correction 
        ev = V-Vd
        
        f3 = open('act_des_position.txt', 'a')
        f3.write("%s,%s,%s,%s,%s,%s\n" % (X[0][0],X[1][0],X[2][0], Xd[0][0],Xd[1][0],Xd[2][0]))

        f4 = open('act_des_vel.txt', 'a')
        f4.write("%s,%s,%s,%s,%s,%s\n" % (V[0][0],V[1][0],V[2][0], Vd[0][0],Vd[1][0],Vd[2][0]))
        """
        # new addition of initegral term
        if self.counter != 0: 
            self.ct = time.time(); dt = self.ct-self.pt
            self.ei = self.ei + ex*dt; self.pt = time.time()
        else: 
            self.pt = time.time()
            self.ei = tp(np.zeros((1,3)))
        
        if ex[0][0] <=0:
            self.a == -1
 
        if ex[1][0] <=0:
            self.b == -1
 
        if ex[2][0] <=0:
            self.c == -1

        self.ki = np.array([[self.a*0.2], [self.b*0.2], [self.c*0.2]])    
        """


        #_b3c = -(mult(self.kx, ex) + mult(self.kv, ev)+mult(self.ki,self.ei))/self.m + self.g*self.e[:,2][np.newaxis].T + ad    
        _b3c = -(mult(self.kx, ex) + mult(self.kv, ev))/self.m + self.g*self.e[:,2][np.newaxis].T + ad # desired direction 
        #print np.linalg.norm(_b3c)
        b3c = ar(_b3c/np.linalg.norm(_b3c)) # get a normalized vector
        #b1c = -tp(np.cross(tp(b3c), np.cross(tp(b3c),tp(b1d))))/np.linalg.norm(np.cross(tp(b3c), tp(b1d))) 
        #b2c = tp(np.cross(tp(b3c), tp(b1c)))
        b2c = tp(np.cross(b3c.T,b1d.T)/np.linalg.norm(np.cross(b3c.T, b1d.T))) # vector b2d 
        b1c = tp(np.cross(b2c.T, b3c.T))
        Rc = np.column_stack((b1c, b2c, b3c)) # desired rotation matrix
        
        f0= open('rotation_desired.txt', 'a')
        f0.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (b1c[0][0], b1c[1][0],b1c[2][0],b2c[0][0],b2c[1][0],b2c[2][0],b3c[0][0],\
        b3c[1][0],b3c[2][0]))

        #ff0= open('rotation_current.txt', 'a')
        #ff0.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (R[0][0],R[1][0],R[2][0],R[0][1],R[1][1],R[2][1],R[0][2],R[1][2],R[2][2]))
        """
        if self.counter != 0:
            delt = time.time()-self.previous_time
            Rc_dot = (Rc-self.Rc_)/delt 
            Rc_ddot = (Rc_dot-self.Rc_dot_)/delt
            self.Rc_ = Rc; self.Rc_dot_ = Rc_dot
            self.previous_time = time.time()
        else: 
            Rc_dot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3))))); self.Rc_ = Rc
            Rc_ddot = np.column_stack((tp(np.zeros((1,3))), tp(np.zeros((1,3))), tp(np.zeros((1,3))))); self.Rc_dot_ = Rc_dot
            self.previous_time = time.time() 
        """
            
        
        if self.counter != 0:
            self.current_time = time.time()
            delt = self.current_time-self.previous_time
            acc = (V-self._V)/delt
            ad_dot = (ad-self._ad)/delt; ad_ddot = (ad_dot-self._ad_dot)/delt   
            V_ddot = (acc-self.acc_)/delt
            self._V = V
            self._ad = ad; self.acc_ = acc; self._ad_dot = ad_dot
            self.previous_time = time.time()
            #f4= open('times.txt', 'a')
            #f4.write("%s,%s,%s\n" % (self.current_time, self.previous_time,delt)) 
        else: 
            self.current_time = time.time()
            acc = tp(np.zeros((1,3)))
            ad_dot = tp(np.zeros((1,3))); self._ad = ad; self._V = V
            V_ddot = tp(np.zeros((1,3))); self.acc_ = acc; ad_ddot = tp(np.zeros((1,3))); self._ad_dot = ad_dot
            self.previous_time = time.time() 
            delt = self.current_time-self.previous_time
        """
        f3 = open('position1.txt', 'a')
        f3.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (X[0][0],X[1][0],X[2][0], V[0][0],V[1][0],V[2][0], \
        acc[0][0],acc[1][0],acc[2][0], V_ddot[0][0],V_ddot[1][0],V_ddot[2][0])) 
   
        
        if self.counter == 0: 
            ad_dot = tp(np.zeros((1,3))); ad_ddot = tp(np.zeros((1,3))); V_ddot = tp(np.zeros((1,3)))
            self.ad_series.append(ad); self.ad_dot_series.append(ad_dot)
            self.acc_series.append(acc) ; self.timeseries.append(time.time())

        elif self.counter == 1:
            delt = time.time()-self.timeseries[-1]
            ad_dot = (ad-self.ad_series[-1])/delt; ad_ddot = (ad_dot-self.ad_dot_series[-1])/delt   
            V_ddot = (acc-self.acc_series[-1])/delt
            self.ad_series.append(ad); self.ad_dot_series.append(ad_dot)
            self.acc_series.append(acc) ; self.timeseries.append(time.time())
             
        else:
            delt = (time.time()-self.timeseries[-2])/2
            
            ad_dot = (3*ad-4*self.ad_series[-1]+self.ad_series[-2])/(2*delt)
            ad_ddot = (3*ad_dot-4*self.ad_dot_series[-1]+self.ad_dot_series[-2])/(2*delt)
            V_ddot = (3*acc-4*self.acc_series[-1]+self.acc_series[-2])/(2*delt)
            self.ad_series.append(ad); self.ad_dot_series.append(ad_dot)
            self.acc_series.append(acc) ; self.timeseries.append(time.time())
            self.ad_series.pop(0); self.ad_dot_series.pop(0); self.acc_series.pop(0); self.timeseries.pop(0)
        """
        f3 = open('position2.txt', 'a')
        f3.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (X[0][0],X[1][0],X[2][0], V[0][0],V[1][0],V[2][0], \
        acc[0][0],acc[1][0],acc[2][0], V_ddot[0][0],V_ddot[1][0],V_ddot[2][0]))
         
        #A = (mult(self.kx, ex) + mult(self.kv, ev)+mult(self.ki, ex))/self.m - self.g*self.e[:,2][np.newaxis].T - ad 
        A = (mult(self.kx, ex) + mult(self.kv, ev))/self.m - self.g*self.e[:,2][np.newaxis].T - ad
        ex_dot =  ev ; ev_dot = acc-ad
        A_dot = (mult(self.kx, ex_dot) + mult(self.kv, ev_dot))/self.m - ad_dot # calculate desired direction 
        b3c_dot = -A_dot/np.linalg.norm(A) + (np.dot(A.T,A_dot)/np.linalg.norm(A)**3)*A
        
        C = tp(np.cross(b3c.T,b1d.T))
        C_dot = tp(np.cross(b3c_dot.T, b1d.T) + np.cross(b3c.T, b1d_dot.T))
        b2c_dot = C_dot/np.linalg.norm(C) - (np.dot(C.T,C_dot)/np.linalg.norm(C)**3)*C
        b1c_dot = tp(np.cross(b2c_dot.T, b3c.T) + np.cross(b2c.T, b3c_dot.T))
        
        Rc_dot = np.column_stack((b1c_dot, b2c_dot, b3c_dot)) # desired rotation matrix derivative
        f5= open('rotation_dot.txt', 'a')   
        f5.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (b1c_dot[0][0], b1c_dot[1][0],b1c_dot[2][0],b2c_dot[0][0],\
        b2c_dot[1][0],b2c_dot[2][0],b3c_dot[0][0],b3c_dot[1][0],b3c_dot[2][0]))        
        
        #Omega_c_hat = np.dot(tp(Rc),Rc_dot) # skew-symmetric desired angular velocity 
        Omega_c_hat = np.dot(Rc.T, Rc_dot)
        #Omega_c_hat = ar([[0,0,0], [0,0,0], [0, 0, 0]])
        Omega_c = ar([[-Omega_c_hat[1][2]], [Omega_c_hat[0][2]], [-Omega_c_hat[0][1]]])

        
        ex_ddot = ev_dot; ev_ddot = V_ddot-ad_dot
        #A_ddot = (mult(self.kx, ex_ddot) + mult(self.kv, ev_ddot)+mult(self.ki, ev))/self.m - ad_ddot
        A_ddot = (mult(self.kx, ex_ddot) + mult(self.kv, ev_ddot))/self.m - ad_ddot
        b3c_ddot = -A_ddot/np.linalg.norm(A) + 2*(np.dot(A.T,A_dot)/np.linalg.norm(A)**3)*A_dot +\
        (np.linalg.norm(A_dot)**2+np.dot(A.T,A_ddot))*A/np.linalg.norm(A)**3 - 3*(np.dot(A.T,A_dot)**2/np.linalg.norm(A)**5)*A
        
        C_ddot = tp(np.cross(b3c_ddot.T, b1d.T) + 2*np.cross(b3c_dot.T, b1d_dot.T) + np.cross(b3c.T, b1d_ddot.T))
        b2c_ddot = C_ddot/np.linalg.norm(C) - 2*(np.dot(C.T,C_dot)/np.linalg.norm(C)**3)*C_dot -\
        (np.linalg.norm(C_dot)**2+np.dot(C.T,C_ddot))*C/np.linalg.norm(C)**3 + 3*(np.dot(C.T,C_dot)**2/np.linalg.norm(C)**5)*C

        b1c_ddot= tp(np.cross(b2c_ddot.T, b3c.T) + 2*np.cross(b2c_dot.T, b3c_dot.T)+np.cross(b2c.T, b3c_ddot.T))

        Rc_ddot = np.column_stack((b1c_ddot, b2c_ddot, b3c_ddot)) # desired rotation matrix derivative
        #f7= open('rotation_ddot.txt', 'a')
        #f7.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (b1c_ddot[0][0], b1c_ddot[1][0],b1c_ddot[2][0],b2c_ddot[0][0],\
        #b2c_ddot[1][0],b2c_ddot[2][0],b3c_ddot[0][0], b3c_ddot[1][0],b3c_ddot[2][0]))
        
        Omega_c_dot_hat = np.dot(Rc.T,Rc_ddot) + np.dot(Omega_c_hat.T,Omega_c_hat) 
        #Omega_c_dot_hat = np.dot(Rc.T,Rc_ddot) - np.linalg.matrix_power(Omega_c_hat,2)
        #Omega_c_dot_hat = ar([[0,0,0], [0,0,0], [0, 0, 0]])
        Omega_c_dot = ar([[-Omega_c_dot_hat[1][2]], [Omega_c_dot_hat[0][2]], [-Omega_c_dot_hat[0][1]]])
        
        eR_hat = 0.5*(np.dot(Rc.T, R) - np.dot(R.T, Rc)) # eR skew symmetric matrix	
        eR = ar([[-eR_hat[1][2]], [eR_hat[0][2]], [-eR_hat[0][1]]]) # vector that gives error in rotation 
        #Omega_c = np.array([[1],[0],[0]])
        eOmega = Omega - np.dot(np.dot(R.T, Rc), Omega_c) # vector that gives error in angular velocity
        #eOmega = Omega - np.dot(np.dot(Rc.T, R), Omega_c) # vector that gives error in angular velocity
        # error function
        #phi = 0.5*np.trace(self.e-np.dot(Rc.T,R))
        #ff2 = open('error_function.txt', 'a')
        #ff2.write("%s\n" % (phi))
        f2 = open('rotation_error.txt', 'a')
        f2.write("%s,%s,%s,%s,%s,%s\n" % (eR[0][0],eR[1][0],eR[2][0], eOmega[0][0],eOmega[1][0],eOmega[2][0]))


        #f8 = open('desired_position.txt', 'a')
        #f8.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (Xd[0][0],Xd[1][0],Xd[2][0], Vd[0][0],Vd[1][0],Vd[2][0], \
        #ad[0][0],ad[1][0],ad[2][0], ad_dot[0][0],ad_dot[1][0],ad_dot[2][0]))
        
        f55= open('some_errors.txt', 'a')   
        f55.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (ex[0][0], ex[1][0],ex[2][0],ev[0][0], ev[1][0],ev[2][0],\
        ev_dot[0][0], ev_dot[1][0],ev_dot[2][0],ev_ddot[0][0], ev_ddot[1][0],ev_ddot[2][0])) 
        
        #Z = np.dot(np.dot(Omega_hat.dot(R.T),Rc),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 
        Z = np.dot(np.dot(Omega_hat,np.dot(R.T,Rc)),Omega_c) - np.dot(np.dot(R.T,Rc), Omega_c_dot) 

        self.f = self.m*np.dot(_b3c.T, tp(R[:,2][np.newaxis]))
        
        #_f = -self.kx*(X-Xd)-self.kv*(V-Vd)+self.m*(self.g*tp(self.e[:,2][np.newaxis]) + ad)
        #self.f = np.dot(tp(_f), tp(R[:,2][np.newaxis]))        
        
        
        #self.M1 = -(mult(self.kR_normalized, eR) + mult(self.kOmega_normalized, eOmega)) + tp(np.cross(Omega.T, Omega.T)) - Z   
        #self.M = np.dot(self.J,self.M1)
        #self.M = tp(np.zeros((1,3)))
        self.M = -(mult(self.kR, eR) + mult(self.kOmega, eOmega)) + tp(np.cross(Omega.T, tp(np.dot(self.J,Omega)))) - np.dot(self.J,Z)
        #print self.M
        """
        Msg = control_inputs()
        Msg.header.stamp = rospy.Time.now()
        Msg.Total_thrust = self.f[0][0]
        Msg.Moment_x = self.M[0][0]
        Msg.Moment_y = self.M[1][0]
        Msg.Moment_z = self.M[2][0]
        
        #f4 = open('forces.txt', 'a')
        #f4.write("%s,%s,%s,%s\n" % (self.f[0][0],self.M[0][0],self.M[1][0], self.M[2][0])
        """
        T = tp(ar([[self.M[0][0],self.M[1][0],self.M[2][0],self.f[0][0]]]))
        
        c1 = tp(ar([[0, -self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c2 = tp(ar([[self.d*self.tau_t, 0, -self.tau_t*self.tau_m, self.tau_t]]))
        c3 = tp(ar([[0, self.d*self.tau_t, self.tau_t*self.tau_m, self.tau_t]]))
        c4 = tp(ar([[-self.d*self.tau_t, 0, -self.tau_t*self.tau_m, self.tau_t]]))
     
        C = np.column_stack((c1,c2,c3,c4)) # solving linear eq T = Cw^2 to get w^2
        w_square = np.dot(np.linalg.inv(C),T)
        w = np.sqrt(np.abs(w_square))
        
        Msg = Actuators()
        Msg.angular_velocities = [w[0][0], w[1][0], w[2][0], w[3][0]]

        #f4 = open('motor_speed.txt', 'a')
        #f4.write("%s,%s,%s,%s\n" % (w[0][0],w[1][0],w[2][0], w[3][0]))   
        
        self.pub.publish(Msg)
        #self.pub.publish(Msg)
        rospy.sleep(0.05)
        self.counter+=1
        b = time.time()
        fa = open('time.txt', 'a')
        fa.write("%s,%s,%s,%s,%s,%s\n" % (a,b,b-a,self.current_time, self.previous_time, delt))
if __name__ == '__main__':

    rospy.init_node('position_controller', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(100)
    start_time = time.time()
    p = parameters("Pelican")
    parameter_list = [p.mass(), p.inertia(), p.kx(), p.kv(), p.kR(), p.gravity(),\
                      p.kOmega(), p.CG_to_CoR(), p.tau_m(), p.tau_t(), p.identity()]

    try: 
        while not rospy.is_shutdown(): 
            current_time = time.time()
            t = current_time-start_time
            print "t=", t 
            c = position_controller("pelican", parameter_list,t)
            r.sleep()

    except rospy.ROSInterruptException(): 
        pass

