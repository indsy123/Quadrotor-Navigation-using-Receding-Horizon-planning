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
import time, operator, scipy
#from mpl_toolkits.mplot3d import Axes3D
#import os, time, operator, json
from gurobipy import *
from functools import reduce
#from multiprocessing import Process
from itertools import chain
from scipy.linalg import block_diag
from mpl_toolkits.mplot3d import Axes3D


class Minimum_snap_trajetory(object):
    def __init__(self, path, waypoint_specified, waypoint_bc): 
        """initializes variables, just does for 4 to start with"""
        self.wp_specfied = waypoint_specified
        self.wp_bc = waypoint_bc
        
        # specify start/intermediate/end points and its deviratives 
        
        
        #path = zip(*path)
        self.x_wp = np.array(path[0])
        self.y_wp = np.array(path[1])
        self.z_wp = np.array(path[2])
        #print self.x_wp, self.y_wp, self.z_wp
        self.no_of_segments = len(self.x_wp)-1
        #print self.no_of_segments
        # enter sequence of waypoints: no of points should be self.no_of_segments+1
        #self.x_wp = np.linspace(0, 2.0, self.no_of_segments+1)
        #self.x_wp = [0, 0.4, 1, 1.6, 2.4, 3]
        # add intial and final condiions vel, acc, jerk
        self.x_ic = np.array([0, 0, 0])
        self.x_fc = np.array([0, 0, 0])
        self.x0 = np.array([self.x_wp[0], self.x_ic[0], self.x_ic[1], self.x_ic[2]])
        self.xT = np.array([self.x_wp[-1], self.x_fc[0], self.x_fc[1], self.x_fc[2]])

        #self.y_wp = np.linspace(0, 2.0, self.no_of_segments+1)
        #self.y_wp = [0, 0.8, 1, 1.7, 2.1, 2.8]
        # add intial and final condiions vel, acc, jek
        self.y_ic = np.array([0, 0, 0])
        self.y_fc = np.array([0, 0, 0])
        self.y0 = np.array([self.y_wp[0], self.y_ic[0], self.y_ic[1], self.y_ic[2]])
        self.yT = np.array([self.y_wp[-1], self.y_fc[0], self.y_fc[1], self.y_fc[2]])

        #self.z_wp = np.linspace(0, 3.0, self.no_of_segments+1)
        #self.z_wp = [0, 0.8, 1.6, 2.4, 3.2, 4]
        # add initial and final conditions vel, acc, jerk
        self.z_ic = np.array([0, 0, 0])
        self.z_fc = np.array([0, 0, 0])
        self.z0 = np.array([self.z_wp[0], self.z_ic[0], self.z_ic[1], self.z_ic[2]])
        self.zT = np.array([self.z_wp[-1], self.z_fc[0], self.z_fc[1], self.z_fc[2]])
        # now enter end times for each segment 
        self.path = [np.sqrt((self.x_wp[i]-self.x_wp[i-1])**2 + (self.y_wp[i]-self.y_wp[i-1])**2 + \
        (self.z_wp[i]-self.z_wp[i-1])**2) for i in range(1, self.no_of_segments+1, 1)]
        
        low_speed = 2.5
        average_speed = 3.0
        self.T = []; self.T.insert(0, 0)
        self.T.insert(1, self.T[-1]+self.path[0]/low_speed)
        for i in range(1, len(self.path)-1, 1):
                self.T.append(self.T[-1]+self.path[i]/average_speed)
        self.T.insert(len(self.T)+1, self.T[-1]+self.path[-1]/low_speed)   

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
        """function that makes vector b of ONLY end points"""
        b = np.concatenate((self.x0, self.xT), axis = 0)
        return b

    def construct_bx(self): 
        """function that makes vector b of ONLY end points"""
        b = np.concatenate((self.x0, self.xT), axis = 0)
        return b

    def construct_by(self): 
        """function that makes vector b of ONLY end points"""
        b = np.concatenate((self.y0, self.yT), axis = 0)
        return b

    def construct_bz(self): 
        """function that makes vector b of ONLY end points"""
        b = np.concatenate((self.z0, self.zT), axis = 0)
        return b
        
    def construct_DF(self, order, r): 
        """function that makes vector DF of ONLY constrained end points"""
        l = [[i] for i in self.x_wp[1:-1]]
        l = list(chain.from_iterable(l))
        DF = np.concatenate((self.x0, l, self.xT), axis = 0)
        return DF
        

    def plot(self, p1, p2, p3): 
        """function to plot position, velocity, acceleration, jerk and snap"""
        #print p, N, n_segments, seq_of_x_wp, T
        N = self.N+1
        p1 = [p1[i:i + N] for i in range(0, len(p1), N)]
        [i.reverse() for i in p1]
    
        p2 = [p2[i:i + N] for i in range(0, len(p2), N)]
        [i.reverse() for i in p2]
    
        p3 = [p3[i:i + N] for i in range(0, len(p3), N)]
        [i.reverse() for i in p3]
        
        t = []; xx = []; vx = []; aax = []; jx = []; sx = []
        yy = []; vy = []; aay = []; jy = []; sy = []
        zz = []; vz = []; aaz = []; jz = []; sz = []
        fig = plt.figure(figsize=(9,6))
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
            ax.plot(xx[i](t[i]), yy[i](t[i]), zz[i](t[i]), zdir='z')

        plt.xlabel('x', fontsize=24)
        plt.ylabel('y', fontsize=24)
        #plt.legend(loc=3, ncol = 3, fontsize=24)
        plt.xticks(size=16); plt.yticks(size=16)     
        fig = plt.figure()
        velocity_magnitude = []; acceleration_magnitude = []; jerk_magnitude = []
        for j in range(self.no_of_segments): 
            velocity_magnitude = np.sqrt(vx[j](t[j])**2 + vy[j](t[j])**2 + vz[j](t[j])**2) 
            acceleration_magnitude = np.sqrt(aax[j](t[j])**2 + aay[j](t[j])**2 + aaz[j](t[j])**2)
            jerk_magnitude = np.sqrt(jx[j](t[j])**2 + jy[j](t[j])**2 + jz[j](t[j])**2)
            plt.plot(t[j], velocity_magnitude, linewidth =2); plt.plot(t[j], acceleration_magnitude, linewidth=2)

        plt.show()
        
    def construct_polynomial_trajectory(self):
        """ function to construct the trajectory"""

        t = time.time()
        print 'p1', time.time()-t
        r = self.r
        N = 1 + self.N # because number of terms in a polynomial = degree+1  

        QQ = []; AA_inv = []

        for i in range(self.no_of_segments): 
            q = self.construct_Q(N, r, self.T[i], self.T[i+1])
            a = self.construct_A(N, r, self.T[i], self.T[i+1])
            a_inv = scipy.linalg.pinv(a)
            QQ = block_diag(QQ, q)
            AA_inv = block_diag(AA_inv, a_inv)

        order = 2*r*self.no_of_segments
        R = np.dot(AA_inv.T, np.dot(QQ, AA_inv))
        bx = self.construct_bx()
        by = self.construct_by()
        bz = self.construct_bz()
        print 'p2', time.time()-t
        m = Model("qp")
        order = 2*r*self.no_of_segments

        dx = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dx")
        dy = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dy")        
        dz = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dz")        

        # making objective using quicksum, takes a lot of time 
        #obj1 = quicksum(dx[i] * quicksum(R[i][j] * dx[j] for j in range(order)) for i in range(order))
        #obj2 = quicksum(dy[i] * quicksum(R[i][j] * dy[j] for j in range(order)) for i in range(order))
        #obj3 = quicksum(dz[i] * quicksum(R[i][j] * dz[j] for j in range(order)) for i in range(order))
        
        # using LinExpr for the second expression is significantly faster 
        obj1 = quicksum(dx[i] * LinExpr([(R[i][j], dx[j]) for j in range(order)]) for i in range(order))
        obj2 = quicksum(dy[i] * LinExpr([(R[i][j], dy[j]) for j in range(order)]) for i in range(order))
        obj3 = quicksum(dz[i] * LinExpr([(R[i][j], dz[j]) for j in range(order)]) for i in range(order))
        print 'p3', time.time()-t
        
        obj = obj1 + obj2 + obj3
        j = 0
        addconstraint = m.addConstr
        for i in range(order):             
            if i < r: 
                addconstraint(dx[i] == bx[i])
                addconstraint(dy[i] == by[i])
                addconstraint(dz[i] == bz[i])
            elif i >= order-r:                 
                addconstraint(dx[i] == bx[r+j])
                addconstraint(dy[i] == by[r+j])
                addconstraint(dz[i] == bz[r+j])
                j += 1

        c = 1 # counter        
        for i in range(r, order-2*r, 2*r): 
            #m.addConstr(dx[i] == self.x_wp[c])
            #m.addConstr(dy[i] == self.y_wp[c])
            #m.addConstr(dz[i] == self.z_wp[c])
            addconstraint(dx[i] <= self.x_wp[c] + 0.05)
            addconstraint(dx[i] >= self.x_wp[c] - 0.05)
            addconstraint(dy[i] <= self.y_wp[c] + 0.05)
            addconstraint(dy[i] >= self.y_wp[c] - 0.05)
            addconstraint(dz[i] <= self.z_wp[c] + 0.05)
            addconstraint(dz[i] >= self.z_wp[c] - 0.05)
            c = c+1
            for j in range(r): 
                addconstraint(dx[i+j] == dx[i+j+r])
                addconstraint(dy[i+j] == dy[i+j+r])
                addconstraint(dz[i+j] == dz[i+j+r])
                #if j ==2: 
                #    m.addConstr(dx[i+j] == 0.5)

        m.setObjective(obj, GRB.MINIMIZE)
        #m.write('model.lp')
        m.setParam('OutputFlag', 0)
        m.setParam('PSDtol', 0.01)

        m.optimize()


        #runtime = m.Runtime
        #optimal_objective = obj.getValue()
        #print 'optimal objective is:', optimal_objective

        x_coeff = [dx[i].X for i in range(order)]
        y_coeff = [dy[i].X for i in range(order)]
        z_coeff = [dz[i].X for i in range(order)]

        Dx = np.asarray(x_coeff)[np.newaxis].T
        Dy = np.asarray(y_coeff)[np.newaxis].T        
        Dz = np.asarray(z_coeff)[np.newaxis].T        
        pcx = np.dot(AA_inv, Dx); pcy = np.dot(AA_inv, Dy); pcz = np.dot(AA_inv, Dz)


        poly_coeff_x = pcx.T.ravel().tolist()
        poly_coeff_y = pcy.T.ravel().tolist()
        poly_coeff_z = pcz.T.ravel().tolist()
        print 'p5', time.time()-t
        return self.T, poly_coeff_x, poly_coeff_y, poly_coeff_z

        #self.plot(poly_coeff_x, poly_coeff_y, poly_coeff_z)


#if __name__ == '__main__':
    #waypoint_specified = True
    #waypoint_bc = False
    #path = np.random.rand(5,3)
    #f = Minimum_snap_trajetory(path, waypoint_specified, waypoint_bc)
    #f.construct_polynomial_trajectory()
