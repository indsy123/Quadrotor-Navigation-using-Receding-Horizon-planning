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
import time, operator, scipy, math
#from mpl_toolkits.mplot3d import Axes3D
#import os, time, operator, json
from gurobipy import *
from functools import reduce
#from multiprocessing import Process
from itertools import chain
from scipy.linalg import block_diag



class Minimum_snap_trajetory(object):
    def __init__(self, planning_counter, vel, path, waypoint_specified, waypoint_bc, v_eoe, a_eoe, j_eoe): 
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

        # add intial and final condiions vel, acc, jerk
        #self.x_ic = np.array([0, 0, 0])
        self.x_ic = np.array([v_eoe[0][0], a_eoe[0][0], j_eoe[0][0]])
        #self.x_fc = np.array([v_eoe[0][0], a_eoe[0][0], j_eoe[0][0]])
        self.x_fc = np.array([0, 0, 0])
        self.x0 = np.array([self.x_wp[0], self.x_ic[0], self.x_ic[1], self.x_ic[2]])
        self.xT = np.array([self.x_wp[-1], self.x_fc[0], self.x_fc[1], self.x_fc[2]])


        # add intial and final condiions vel, acc, jerk
        #self.y_ic = np.array([0, 0, 0])
        self.y_ic = np.array([v_eoe[0][1], a_eoe[0][1], j_eoe[0][1]])
        #self.y_fc = np.array([v_eoe[0][1], a_eoe[0][1], j_eoe[0][1]])
        self.y_fc = np.array([0, 0, 0])
        self.y0 = np.array([self.y_wp[0], self.y_ic[0], self.y_ic[1], self.y_ic[2]])
        self.yT = np.array([self.y_wp[-1], self.y_fc[0], self.y_fc[1], self.y_fc[2]])


        # add initial and final conditions vel, acc, jerk
        #self.z_ic = np.array([0, 0, 0])
        self.z_ic = np.array([v_eoe[0][2], a_eoe[0][2], j_eoe[0][2]])
        #self.z_fc = np.array([v_eoe[0][2], a_eoe[0][2], j_eoe[0][2]])
        self.z_fc = np.array([0, 0, 0])
        self.z0 = np.array([self.z_wp[0], self.z_ic[0], self.z_ic[1], self.z_ic[2]])
        self.zT = np.array([self.z_wp[-1], self.z_fc[0], self.z_fc[1], self.z_fc[2]])
        
        
        a = [self.x_wp[i+1]-self.x_wp[i] for i in range(len(self.x_wp)-1)]
        b = [self.y_wp[i+1]-self.y_wp[i] for i in range(len(self.y_wp)-1)]
        c = [self.z_wp[i+1]-self.z_wp[i] for i in range(len(self.z_wp)-1)]
        self.path = [np.sqrt(i**2+j**2+k**2) for (i, j, k) in zip(a,b,c)] # length is equal to number of segments   

        """
        self.T = []; self.T.insert(0, 0)        
        # first step when the quad goes to hover
        if planning_counter == -1: 
            self.T.insert(1, self.path[0]/vel) 
        # when there is no path     
        if len(self.path) == 0: 
            print 'there is no path..'

        if planning_counter != -1:
            for i in range(len(self.path)):
                self.T.append(self.T[-1]+self.path[i]/vel)


        """
        self.T = []
        self.T.insert(0, 0)
        vel = vel
        # first step when the quad goes to hover
        if planning_counter == -1: 
            self.T.insert(1, self.path[0]/vel) 
        # when there is no path     
        if len(self.path) == 0: 
            print 'there is no path..'
        # start defining velocity profile
        v_av = [0]*(len(self.path)+1)

        if len(self.path) >= 4: 
            if planning_counter == 0: 
                v_av[0] = 0; v_av[1] = 0.5*vel
                for i in range(2, len(self.path)-1): 
                    v_av[i] = vel
                v_av[-2] = 0.5*vel; v_av[-1] = 0
    
                for i in range(len(self.path)): 
                    self.T.append(self.T[i]+2*self.path[i]/(v_av[i+1]+v_av[i]))
            elif planning_counter == 1: 
                v_av[0] = 0.5*vel; v_av[1] = vel
                for i in range(2, len(self.path)-1): 
                    v_av[i] = vel
                v_av[-2] = 0.5*vel; v_av[-1] = 0
                for i in range(len(self.path)): 
                    self.T.append(self.T[i]+2*self.path[i]/(v_av[i+1]+v_av[i]))
            elif planning_counter > 1: 
                v_av[0] = vel; v_av[1] = vel
                for i in range(2, len(self.path)-1): 
                    v_av[i] = vel
                v_av[-2] = 0.5*vel; v_av[-1] = 0
                for i in range(len(self.path)): 
                    self.T.append(self.T[i]+2*self.path[i]/(v_av[i+1]+v_av[i]))

        self.r = 4 # corresponding to snap which is 4th derivative
        self.N = 7# degree of polynomial 

        
    def prod(self, iterable):
        """multiply all iterables"""
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
                if i >= r and l >=r:  
                    q[i,l] = 2*self.prodoftwoterms(i,l,r)*(math.pow(tf, i+l-2*r+1) - math.pow(t0, i+l-2*r+1))/(i+l-2*r+1)
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

        r = self.r
        N = 1 + self.N # because number of terms in a polynomial = degree+1  

        QQ = []; AA_inv = []

        for i in range(self.no_of_segments): 
            q = self.construct_Q(N, r, self.T[i], self.T[i+1])
            a = self.construct_A(N, r, self.T[i], self.T[i+1])
            a_inv = scipy.linalg.pinv(a)
            QQ = block_diag(QQ, q)
            AA_inv = block_diag(AA_inv, a_inv)

            
        # for some weird reason, the new version of scipy adds an extra first row (all zero) when I make the block diadonal matrix in loop, but the nuc on this diy has old version so next two lines are suppressed
        #QQ = QQ[1:N*self.no_of_segments+1]
        #AA_inv = AA_inv[1:N*self.no_of_segments+1]
        order = 2*r*self.no_of_segments
        R = np.dot(AA_inv.T, np.dot(QQ, AA_inv))
        
        bx = np.concatenate((self.x0, self.xT), axis = 0)
        by = np.concatenate((self.y0, self.yT), axis = 0)
        bz = np.concatenate((self.z0, self.zT), axis = 0)

        m = Model("qp")
        order = 2*r*self.no_of_segments

        dx = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dx")
        dy = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dy")        
        dz = m.addVars(order, lb = -GRB.INFINITY, ub = GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dz")        

       
        # using LinExpr for the second expression is significantly faster 
        obj1 = quicksum(dx[i] * LinExpr([(R[i][j], dx[j]) for j in range(order)]) for i in range(order))
        obj2 = quicksum(dy[i] * LinExpr([(R[i][j], dy[j]) for j in range(order)]) for i in range(order))
        obj3 = quicksum(dz[i] * LinExpr([(R[i][j], dz[j]) for j in range(order)]) for i in range(order))

        
        obj = obj1 + obj2 + obj3
        j = 0
        addconstraint = m.addConstr
        for k in range(order):             
            if k < r: 
                addconstraint(dx[k] == bx[k])
                addconstraint(dy[k] == by[k])
                addconstraint(dz[k] == bz[k])
            elif k >= order-r:                 
                addconstraint(dx[k] == bx[r+j])
                addconstraint(dy[k] == by[r+j])
                addconstraint(dz[k] == bz[r+j])
                j += 1

        c = 1 # counter        
        for n in range(r, order-2*r, 2*r): 
            #if c ==3: 
            #m.addConstr(dx[n] == self.x_wp[c])
            #m.addConstr(dy[n] == self.y_wp[c])
            #m.addConstr(dz[n] == self.z_wp[c])
            #else: 
            m.addConstr(dx[n] <= self.x_wp[c] + 0.05)
            m.addConstr(dy[n] <= self.y_wp[c] + 0.05)
            m.addConstr(dz[n] <= self.z_wp[c] + 0.05)
            m.addConstr(dx[n] >= self.x_wp[c] - 0.05)
            m.addConstr(dy[n] >= self.y_wp[c] - 0.05)
            m.addConstr(dz[n] >= self.z_wp[c] - 0.05)
            c = c+1
            for q in range(r): 
                addconstraint(dx[n+q] == dx[n+q+r])
                addconstraint(dy[n+q] == dy[n+q+r])
                addconstraint(dz[n+q] == dz[n+q+r])
                #addconstraint(dx[n+1] == 0)
                #addconstraint(dy[n+1] == 0)
                #addconstraint(dz[n+1] == 0)
                #addconstraint(dx[n+2] == 0)
                #addconstraint(dy[n+2] == 0)
                #addconstraint(dz[n+2] == 0)
                #addconstraint(dx[n+3] == 0)
                #addconstraint(dy[n+3] == 0)
                #addconstraint(dz[n+3] == 0)


        m.setObjective(obj, GRB.MINIMIZE)
        #m.write('model.lp')
        m.setParam('OutputFlag', 0)
        m.setParam('PSDtol', 1e5)
        m.setParam('NumericFocus', 3)

        m.optimize()
        print 'The optimality status is (2: optimal, 13: suboptimal):', m.status

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

        return self.T, poly_coeff_x, poly_coeff_y, poly_coeff_z

        #self.plot(poly_coeff_x, poly_coeff_y, poly_coeff_z)


#if __name__ == '__main__':
    #waypoint_specified = True
    #waypoint_bc = False
    #path = np.random.rand(5,3)
    #f = Minimum_snap_trajetory(path, waypoint_specified, waypoint_bc)
    #f.construct_polynomial_trajectory()
