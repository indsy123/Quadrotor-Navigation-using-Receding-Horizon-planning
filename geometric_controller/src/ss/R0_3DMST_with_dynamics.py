#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  5 14:01:09 2019
Minimum snap trajectory generation algorithm based on Rithcer's paper 
R1: takes multiple segments
@author: indrajeet
"""

import numpy as np 
from matplotlib import pyplot as plt 
#import random 
#from mpl_toolkits.mplot3d import Axes3D
import time, operator
from gurobipy import *
from functools import reduce
import scipy
#from scipy import linalg
#from multiprocessing import Process

class Minimum_snap_trajetory(object):
    def __init__(self, vel, path, waypoint_specified, waypoint_bc, v_eoe, a_eoe, j_eoe): 
        """initializes variables, just does for 4 to start with"""
        self.start_time = time.time()
        self.wp_specfied = waypoint_specified
        self.wp_bc = waypoint_bc
        # specify start/intermediate/end points and its deviratives 
        self.x_wp = np.array(path[0])
        self.y_wp = np.array(path[1])
        self.z_wp = np.array(path[2])
        
        self.no_of_segments = len(self.x_wp)-1
        self.max_velocity = 4.0


        # add intial and final condiions vel, acc, jerk
        self.x_ic = np.array([v_eoe[0][0], a_eoe[0][0], j_eoe[0][0]])
        #self.x_ic = np.array([0, 0, 0])
        self.x_fc = np.array([0, 0, 0])# final velocity can be same as initial velocity with both acceleration and jerk being zero, think!!
        self.x0 = np.array([self.x_wp[0], self.x_ic[0], self.x_ic[1], self.x_ic[2]])
        self.xT = np.array([self.x_wp[-1], self.x_fc[0], self.x_fc[1], self.x_fc[2]])


        # add intial and final condiions vel, acc, jerk
        self.y_ic = np.array([v_eoe[0][1], a_eoe[0][1], j_eoe[0][1]])
        #self.y_ic = np.array([0, 0, 0])
        self.y_fc = np.array([0, 0, 0])# final velocity can be same as initial velocity with both acceleration and jerk being zero, think!!
        self.y0 = np.array([self.y_wp[0], self.y_ic[0], self.y_ic[1], self.y_ic[2]])
        self.yT = np.array([self.y_wp[-1], self.y_fc[0], self.y_fc[1], self.y_fc[2]])


        # add initial and final conditions vel, acc, jerk
        self.z_ic = np.array([v_eoe[0][2], a_eoe[0][2], j_eoe[0][2]])
        #self.z_ic = np.array([0, 0, 0])
        self.z_fc = np.array([0, 0, 0])# final velocity can be same as initial velocity with both acceleration and jerk being zero, think!!
        self.z0 = np.array([self.z_wp[0], self.z_ic[0], self.z_ic[1], self.z_ic[2]])
        self.zT = np.array([self.z_wp[-1], self.z_fc[0], self.z_fc[1], self.z_fc[2]])
        
        # now enter end times for each segment
        a = [self.x_wp[i+1]-self.x_wp[i] for i in range(len(self.x_wp)-1)]
        b = [self.y_wp[i+1]-self.y_wp[i] for i in range(len(self.y_wp)-1)]
        c = [self.z_wp[i+1]-self.z_wp[i] for i in range(len(self.z_wp)-1)]
        self.path = [np.sqrt(i**2+j**2+k**2) for (i, j, k) in zip(a,b,c)]
        
      
        #self.V_average = 2.0 # velocity in m/s
        self.T = []; self.T.insert(0, 0)
        #self.T = []

        for i in range(len(self.path)):
                self.T.append(self.T[-1]+self.path[i]/vel)
        #self.T = [i/self.T[-1] for i in self.T]
        print self.path, self.T
        self.r = 4 # corresponding to snap which is 4th derivative
        self.N = 7# degree of polynomial 
        
    def prod(self, iterable):
        return reduce(operator.mul, iterable, 1)
         
    def prodoftwoterms(self, ii, ll, r): 
        """fucntion to iterate the product of two terms"""
        prod = 1
        for k in range(r):
            prod = prod * (ii-k)*(ll-k) 
        return prod

    
    def construct_Q(self, N, r, t0, tf): 
        """function to constructs the Q matrix"""
        q = np.zeros((N, N))
        for i in xrange(N): 
            for l in xrange(N):
                if i>= r and l>=r:  
                    q[i,l] = 2*self.prodoftwoterms(i,l,r)*(tf**(i+l-2*r+1) - t0**(i+l-2*r+1))/(i+l-2*r+1)
                    #q[i,l] = 2*self.prodoftwoterms(i,l,r)*((tf-t0)**(i+l-2*r+1))/(i+l-2*r+1)
                else: 
                    q[i,l] = 0
        return q
      
    def construct_A(self, N, r, t0, tf): 
        """function to construct the A matrix"""
        a = np.zeros((2*r,N))
                   
        for i in range(N): 
            for j in range(r): 
                if i >= j : 
                    a[j,i] = self.prod(xrange(i-(j-1), i+1))*t0**(i-j)
                    a[r+j,i] = self.prod(xrange(i-(j-1), i+1))*tf**(i-j)
                         
        return a
    
    def construct_b(self): 
        """function that makes vector b of end points"""
        b = np.concatenate((self.x0, self.xT), axis = 0)
        return b
        
    def plot_trajectory(self, p1, p2, p3, N): 
        """function to plot position, velocity, acceleration, jerk and snap"""
        #print 'p1', p1
        p1 = [p1[i:i + N] for i in range(0, len(p1), N)]
        [i.reverse() for i in p1]
        #print 'p1 again', p1
        p2 = [p2[i:i + N] for i in range(0, len(p2), N)]
        [i.reverse() for i in p2]
    
        p3 = [p3[i:i + N] for i in range(0, len(p3), N)]
        [i.reverse() for i in p3]
        
        t = []; xx = []; vx = []; aax = []; jx = []; sx = []
        yy = []; vy = []; aay = []; jy = []; sy = []
        zz = []; vz = []; aaz = []; jz = []; sz = []
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        for i in range(self.no_of_segments): 
            t.append(np.linspace(self.T[i], self.T[i+1], 101))   
            xx.append(np.poly1d(p1[i]))
            vx.append(np.polyder(xx[-1], 1)); aax.append(np.polyder(xx[-1], 2))
            jx.append(np.polyder(xx[-1], 3)); sx.append(np.polyder(xx[-1], 4))
            
            yy.append(np.poly1d(p2[i]))
            vy.append(np.polyder(yy[-1], 1)); aay.append(np.polyder(yy[-1], 2))
            jy.append(np.polyder(yy[-1], 3)); sy.append(np.polyder(yy[-1], 4))
    
            zz.append(np.poly1d(p3[i]))
            vz.append(np.polyder(zz[-1], 1)); aaz.append(np.polyder(zz[-1], 2))
            jz.append(np.polyder(zz[-1], 3)); sz.append(np.polyder(zz[-1], 4))

        for i in range(self.no_of_segments): 
            #x_final.append(xx[i](t[i]))
            ax.plot(xx[i](t[i]), yy[i](t[i]), zz[i](t[i]), zdir='z')
            
        plt.xlabel('x', fontsize=24)
        plt.ylabel('y', fontsize=24)
        #plt.legend(loc=3, ncol = 3, fontsize=24)
        plt.xticks(size=16); plt.yticks(size=16)
        plt.xlim(-5, 5); plt.ylim(-5, 5)
        fig = plt.figure('velocity')
        
        for j in range(self.no_of_segments): 
            
            plt.plot(t[j], vx[j](t[j]), linewidth =2, color = 'r')
            #plt.plot(t[j][:-1], vx_final[j], linewidth =2, color = 'b')
            plt.plot(t[j], vy[j](t[j]), color = 'g', linewidth =2)
            plt.plot(t[j], vz[j](t[j]), color = 'b', linewidth =2)

        fig = plt.figure('acceleration')
        
        for j in range(self.no_of_segments): 
            plt.plot(t[j], aax[j](t[j]), linewidth =2, color = 'r')
            plt.plot(t[j], aay[j](t[j]), color = 'g', linewidth =2)
            plt.plot(t[j], aaz[j](t[j]), color = 'b', linewidth =2)
            
        fig = plt.figure('position')
        
        for j in range(self.no_of_segments): 
            plt.plot(t[j], xx[j](t[j]), linewidth =2, color = 'r')
            plt.plot(t[j], yy[j](t[j]), color = 'g', linewidth =2)
            plt.plot(t[j], zz[j](t[j]), color = 'b', linewidth =2)
        
        for i in range(1, self.no_of_segments): 
            plt.vlines(self.T[i], 0, self.x_wp[-1], 'k')
            plt.hlines(self.x_wp[i], 0, self.T[-1], 'k')
            plt.hlines(self.y_wp[i], 0, self.T[-1], 'k', linestyle='--')
            plt.hlines(self.z_wp[i], 0, self.T[-1], 'k', linestyle=':')
        
        fig = plt.figure()
        velocity_magnitude = []; acceleration_magnitude = []; jerk_magnitude = []
        snap_magnitude = []
        for j in range(self.no_of_segments): 
            velocity_magnitude = np.sqrt(vx[j](t[j])**2 + vy[j](t[j])**2 + vz[j](t[j])**2) 
            acceleration_magnitude = np.sqrt(aax[j](t[j])**2 + aay[j](t[j])**2 + aaz[j](t[j])**2)
            jerk_magnitude = np.sqrt(jx[j](t[j])**2 + jy[j](t[j])**2 + jz[j](t[j])**2)
            snap_magnitude = np.sqrt(sx[j](t[j])**2 + sy[j](t[j])**2 + sz[j](t[j])**2)
            plt.plot(t[j], velocity_magnitude, linewidth =2, color = 'r'); plt.plot(t[j], acceleration_magnitude, linewidth=2, color='g')
            #plt.plot(t[j], jerk_magnitude, linewidth=2, color = 'b'); #plt.plot(t[j], snap_magnitude, linewidth=2, color='m')

        plt.show()


        
    def construct_polynomial_trajectory(self):
        """ function to construct the trajectory"""

        r = self.r
        N = 1 + self.N # because no of terms in a polynomial = degree+1  

        Q_bar = []; A_bar = []; QQ = []; AA = []
        
        for i in range(self.no_of_segments): 
            Q_bar.append(self.construct_Q(N, r, self.T[i], self.T[i+1]))
            A_bar.append(self.construct_A(N, r, self.T[i], self.T[i+1]))
            QQ = scipy.linalg.block_diag(QQ, Q_bar[-1])
            AA = scipy.linalg.block_diag(AA, A_bar[-1])

        np.set_printoptions(suppress=True)
        # take the part of A out that represents boundary conditions, first r rows of A_bar[0] and last r rows of A_bar[-1]
        # if there are just two waypoints, A_boundaryconditions = A_bar
        A_boundaryconditions = np.concatenate((A_bar[0][:r], A_bar[-1][r:]))


        bx = np.concatenate((self.x0, self.xT), axis = 0)
        by = np.concatenate((self.y0, self.yT), axis = 0)
        bz = np.concatenate((self.z0, self.zT), axis = 0)
        


        m = Model("qp")
        order = 2*r*self.no_of_segments

        px = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="px")
        py = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="py")
        pz = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="pz")       

       
        # using LinExpr for the second expression is significantly faster 
        obj1 = quicksum(px[i] * LinExpr([(QQ[i][j], px[j]) for j in range(order)]) for i in range(order))
        obj2 = quicksum(py[i] * LinExpr([(QQ[i][j], py[j]) for j in range(order)]) for i in range(order))
        obj3 = quicksum(pz[i] * LinExpr([(QQ[i][j], pz[j]) for j in range(order)]) for i in range(order))

        
        obj = obj1 + obj2 + obj3 

        

        m.setObjective(obj, GRB.MINIMIZE)
        
        for i in range(2*r): 
            if i < r: 
                m.addConstr(quicksum(A_boundaryconditions[i,j]*px[j] for j in range(N)) == bx[i])
                m.addConstr(quicksum(A_boundaryconditions[i,j]*py[j] for j in range(N)) == by[i])
                m.addConstr(quicksum(A_boundaryconditions[i,j]*pz[j] for j in range(N)) == bz[i])
            else: 
                m.addConstr(quicksum(A_boundaryconditions[i,j]*px[j+(self.no_of_segments-1)*N] for j in range(N)) == bx[i])
                m.addConstr(quicksum(A_boundaryconditions[i,j]*py[j+(self.no_of_segments-1)*N] for j in range(N)) == by[i])
                m.addConstr(quicksum(A_boundaryconditions[i,j]*pz[j+(self.no_of_segments-1)*N] for j in range(N)) == bz[i])

        if self.no_of_segments >= 1 and self.wp_bc == False:  
            for i in range(self.no_of_segments):
                ii = i+1
                for j in range(r): 
                    if ii < self.no_of_segments: 
                        m.addConstr(quicksum(A_bar[i][j+r][k]*px[k+i*N] for k in range(N)) == quicksum(A_bar[ii][j][k]*px[k+ii*N] for k in range(N)))
                        m.addConstr(quicksum(A_bar[i][j+r][k]*py[k+i*N] for k in range(N)) == quicksum(A_bar[ii][j][k]*py[k+ii*N] for k in range(N)))
                        m.addConstr(quicksum(A_bar[i][j+r][k]*pz[k+i*N] for k in range(N)) == quicksum(A_bar[ii][j][k]*pz[k+ii*N] for k in range(N)))
           
        if self.wp_specfied == True: 
            for i in range(len(self.x_wp[1:-1])):
                m.addConstr(quicksum(A_bar[i][r][k]*px[k+i*N] for k in range(N)) == self.x_wp[i+1])
                m.addConstr(quicksum(A_bar[i][r][k]*py[k+i*N] for k in range(N)) == self.y_wp[i+1])
                m.addConstr(quicksum(A_bar[i][r][k]*pz[k+i*N] for k in range(N)) == self.z_wp[i+1])
        
        
        #subsample = 10
        #for k in range(self.no_of_segments): 
        #    ttt = np.linspace(self.T[k], self.T[k+1], subsample).ravel()
        #    for t in ttt[1:-1]: 
        #        a = px[k*N+1] + 2*t*px[k*N+2] + 3*t*t*px[k*N+3] + 4*t*t*t*px[k*N+4] + 5*t*t*t*t*px[k*N+5] + 6*t*t*t*t*t*px[k*N+6] + 7*t*t*t*t*t*t*px[k*N+7]
        #        b = py[k*N+1] + 2*t*py[k*N+2] + 3*t*t*py[k*N+3] + 4*t*t*t*py[k*N+4] + 5*t*t*t*t*py[k*N+5] + 6*t*t*t*t*t*py[k*N+6] + 7*t*t*t*t*t*t*py[k*N+7]
        #        c = pz[k*N+1] + 2*t*pz[k*N+2] + 3*t*t*pz[k*N+3] + 4*t*t*t*pz[k*N+4] + 5*t*t*t*t*pz[k*N+5] + 6*t*t*t*t*t*pz[k*N+6] + 7*t*t*t*t*t*t*pz[k*N+7]
        #        m.addConstr(a*a + b*b + c*c <= self.max_velocity*self.max_velocity)
        
          
        #m.write('model_qp.lp')
        m.setParam('OutputFlag', 0)
        m.setParam('NumericFocus', 3)
        m.setParam('PSDtol', 1e3)
        m.optimize()
        #runtime = m.Runtime
        #m.printQuality()
        #print "The run time is:", runtime
        #coeff_x = []; coeff_y = []; coeff_z = []
        
        poly_coeff_x = [px[i].X for i in range(order)]
        poly_coeff_y = [py[i].X for i in range(order)]
        poly_coeff_z = [pz[i].X for i in range(order)]
        return self.T, poly_coeff_x, poly_coeff_y, poly_coeff_z
        #for i in range(order): 
        #    coeff_x.append(px[i].X); coeff_y.append(py[i].X); coeff_z.append(pz[i].X)
        #print 'total time taken', time.time()-self.start_time

        #self.plot_trajectory(coeff_x, coeff_y, coeff_z, N)

#if __name__ == '__main__':
#    waypoint_specified = True
#    waypoint_bc = False
#    f = Minimum_snap_trajetory(waypoint_specified, waypoint_bc)
#    f.construct_polynomial_trajectory()
