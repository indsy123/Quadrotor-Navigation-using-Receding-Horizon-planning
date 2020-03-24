#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  5 14:01:09 2019
Polynomial trajectory generation based on Mellinger's paper
@author: indrajeet
"""

import numpy as np 
#from matplotlib import pyplot as plt 
import time, operator, scipy, math
#from mpl_toolkits.mplot3d import Axes3D
#import os, time, operator, json
from gurobipy import *
from functools import reduce
#from multiprocessing import Process
from itertools import chain
from scipy.linalg import block_diag
from scipy.special import erf


class Minimum_jerk_trajetory(object):
    def __init__(self, counter, vel, path, v_eoe, a_eoe, j_eoe): 
        """initializes variables, just does for 4 to start with"""

        path = zip(*path)
        self.x_wp = np.array(path[0])
        self.y_wp = np.array(path[1])
        self.z_wp = np.array(path[2])

        self.no_of_segments = len(self.x_wp)-1

        self.x_ic = np.array([v_eoe[0], a_eoe[0], j_eoe[0]])
        self.x_fc = np.array([0, 0, 0])
        self.x0 = np.array([self.x_wp[0], self.x_ic[0], self.x_ic[1], self.x_ic[2]])
        self.xT = np.array([self.x_wp[-1], self.x_fc[0], self.x_fc[1], self.x_fc[2]])

        self.y_ic = np.array([v_eoe[1], a_eoe[1], j_eoe[1]])
        self.y_fc = np.array([0, 0, 0])
        self.y0 = np.array([self.y_wp[0], self.y_ic[0], self.y_ic[1], self.y_ic[2]])
        self.yT = np.array([self.y_wp[-1], self.y_fc[0], self.y_fc[1], self.y_fc[2]])


        self.z_ic = np.array([v_eoe[2], a_eoe[2], j_eoe[2]])
        self.z_fc = np.array([0, 0, 0])
        self.z0 = np.array([self.z_wp[0], self.z_ic[0], self.z_ic[1], self.z_ic[2]])
        self.zT = np.array([self.z_wp[-1], self.z_fc[0], self.z_fc[1], self.z_fc[2]])

        a = [self.x_wp[i+1]-self.x_wp[i] for i in range(len(self.x_wp)-1)]
        b = [self.y_wp[i+1]-self.y_wp[i] for i in range(len(self.y_wp)-1)]
        c = [self.z_wp[i+1]-self.z_wp[i] for i in range(len(self.z_wp)-1)]
        self.path = [np.sqrt(i*i+j*j+k*k) for (i, j, k) in zip(a,b,c)] 

        self.T = []
        self.T.insert(0, 0)
        # first step when the quad goes to hover
        
        if counter == 0: 
            self.T.insert(1, self.path[0]/vel)
        """    
        if counter == 1: 
            for i in range(len(self.path)): 
                if i == 0 or i == 1 or i == len(self.path)-2 or i == len(self.path)-1: 
                    self.T.insert(i+1, self.T[i]+self.path[i]/(vel*0.5))
                else: 
                    self.T.insert(i+1, self.T[i]+self.path[i]/vel)
        elif counter == 2: 
            for i in range(len(self.path)): 
                if i == 0 or i == len(self.path)-2 or i == len(self.path)-1: 
                    self.T.insert(i+1, self.T[i]+self.path[i]/(vel*0.5))
                else: 
                    self.T.insert(i+1, self.T[i]+self.path[i]/vel)   
        elif counter > 2: 
            for i in range(len(self.path)): 
                if i == len(self.path)-2 or i == len(self.path)-1: 
                    self.T.insert(i+1, self.T[i]+self.path[i]/(vel*0.5))
                else: 
                    self.T.insert(i+1, self.T[i]+self.path[i]/vel) 
        
        """
        v_av = [0]*(len(self.path)+1)

        if len(self.path) >= 4: 
            if counter == 1: 
                v_av[0] = 0; v_av[1] = 0.5*vel
                for i in range(2, len(self.path)-1): 
                    v_av[i] = vel
                v_av[-2] = 0.5*vel; v_av[-1] = 0
    
                for i in range(len(self.path)): 
                    self.T.append(self.T[i]+2*self.path[i]/(v_av[i+1]+v_av[i]))
            elif counter == 2: 
                v_av[0] = 0.5*vel; v_av[1] = vel
                for i in range(2, len(self.path)-1): 
                    v_av[i] = vel
                v_av[-2] = 0.5*vel; v_av[-1] = 0
                for i in range(len(self.path)): 
                    self.T.append(self.T[i]+2*self.path[i]/(v_av[i+1]+v_av[i]))
            elif counter > 2: 
                v_av[0] = vel; v_av[1] = vel
                for i in range(2, len(self.path)-1): 
                    v_av[i] = vel
                v_av[-2] = 0.5*vel; v_av[-1] = 0
                for i in range(len(self.path)): 
                    self.T.append(self.T[i]+2*self.path[i]/(v_av[i+1]+v_av[i]))

        # kept fix for now
        self.r = 3 
        self.N = 5
        
    def prod(self, iterable):
        """multiply all iterables"""
        return reduce(operator.mul, iterable, 1)
         
    def prodoftwoterms(self, ii, ll, r): 
        """fucntion to iterate the product of two terms"""
        prod = 1
        for k in range(r):
            prod = prod * (ii-k)*(ll-k) 
        return prod

    def construct_Q(self, N, r, T, ns): 
        """function to constructs the Q matrix"""
        #print N, r
        q = np.zeros((N, N)); QQ = []
        for k in range(self.no_of_segments): 
            for i in xrange(N): 
                for l in xrange(N):
                    if i>= r and l>=r:   
                        q[i,l] = 2*self.prodoftwoterms(i,l,r)*(math.pow(T[k+1], i+l-2*r+1) - math.pow(T[k], i+l-2*r+1))/(i+l-2*r+1)

            QQ = block_diag(QQ, q)

        return QQ
      
    def construct_A(self, N, r, T, ns): 
        """function to construct the A matrix"""
        a = np.zeros((2*r,N)); AA = []
        for k in range(self.no_of_segments):            
            for i in range(N): 
                for j in range(r): 
                    if i >= j : 
                        a[j,i] = self.prod(xrange(i-(j-1), i+1))*T[k]**(i-j)
                        a[r+j,i] = self.prod(xrange(i-(j-1), i+1))*T[k+1]**(i-j)

            AA = block_diag(AA, a)

                       
        return AA
        
    def construct_trajectory(self):
        """ function to construct the trajectory"""
        t = time.time()
        r = self.r
        N = 1 + self.N # because number of terms in a polynomial = degree+1  

        QQ = self.construct_Q(N, r, self.T, self.no_of_segments)
        AA = self.construct_A(N, r, self.T, self.no_of_segments)

        AA_inv = scipy.linalg.pinv(AA)

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


        obj1 = quicksum(dx[i] * LinExpr([(R[i][j], dx[j]) for j in range(order)]) for i in range(order))
        obj2 = quicksum(dy[i] * LinExpr([(R[i][j], dy[j]) for j in range(order)]) for i in range(order))
        obj3 = quicksum(dz[i] * LinExpr([(R[i][j], dz[j]) for j in range(order)]) for i in range(order))


        obj = obj1 + obj2 + obj3
        j = 0
        m.addConstrs(dx[k] == bx[k] for k in range(order) if k<r)
        m.addConstrs(dy[k] == by[k] for k in range(order) if k<r)
        m.addConstrs(dz[k] == bz[k] for k in range(order) if k<r)
        
        for k in range(order):
            if k >= order-r:     
                # min snap
                #m.addConstr(dx[k] == bx[r+j])
                #m.addConstr(dy[k] == by[r+j])
                #m.addConstr(dz[k] == bz[r+j])
                # min jerk
                m.addConstr(dx[k] == bx[r+j+1])
                m.addConstr(dy[k] == by[r+j+1])
                m.addConstr(dz[k] == bz[r+j+1])
                j += 1

        c = 1 # counter        
        for n in range(r, order-2*r, 2*r): 
            m.addConstr(dx[n] <= self.x_wp[c] + 0.1)
            m.addConstr(dy[n] <= self.y_wp[c] + 0.1)
            m.addConstr(dz[n] <= self.z_wp[c] + 0.1)
            m.addConstr(dx[n] >= self.x_wp[c] - 0.1)
            m.addConstr(dy[n] >= self.y_wp[c] - 0.1)
            m.addConstr(dz[n] >= self.z_wp[c] - 0.1)
            c = c+1
            m.addConstrs(dx[n+q] == dx[n+q+r] for q in range(r))
            m.addConstrs(dy[n+q] == dy[n+q+r] for q in range(r))
            m.addConstrs(dz[n+q] == dz[n+q+r] for q in range(r))


        m.setObjective(obj, GRB.MINIMIZE)
        #m.write('model.lp')
        m.setParam('OutputFlag', 0)
        m.setParam('PSDtol', 1e6)
        m.setParam('NumericFocus', 3)

        m.optimize()


        Dx = np.array([dx[i].X for i in range(order)])[np.newaxis].T
        Dy = np.array([dy[i].X for i in range(order)])[np.newaxis].T
        Dz = np.array([dz[i].X for i in range(order)])[np.newaxis].T       
        pcx = np.dot(AA_inv, Dx); pcy = np.dot(AA_inv, Dy); pcz = np.dot(AA_inv, Dz)

        poly_coeff_x = pcx.T.ravel().tolist()
        poly_coeff_y = pcy.T.ravel().tolist()
        poly_coeff_z = pcz.T.ravel().tolist()

        return self.T, poly_coeff_x, poly_coeff_y, poly_coeff_z

