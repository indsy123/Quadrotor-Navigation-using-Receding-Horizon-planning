#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 11:17:34 2017
Defines all the parameters of various acstec helicopters measured from testing
in the lab. Unless otherwise mentioned the units are SI. however, I may change 
them as I develop this script, will try to write as much as I can whereever 
there is a deviation from SI units.  
defined parameters are as follows: 
mass: mass of the vehicle
inertia: moment of inertia of the vehicle 
CG_to_CoR: distance between center of gravity of the vehicle to the center of 
rotation of the propeller 
kx: gain of the position error 
kv: gain of the velocity error 
kR: gain of the rotation error 
kOmega: gain of the angular velocity error 
tau_t: thrust constant of the propeller (assuming f = tau_t*w^2)
tau_m: moment constant of the propeller   
change the diagonal entries in inertia and gain matrices to change the values 
in x, y and z direction. off-diagonal entries should be zero. May have 
off-diagonal terms in inertia matrix if there is a reason to do so later. 
Somehow this long thing is working but its stupid. need to find a way to make 
it short by calling self.uav outside the function definition.  
"""
__author__ = 'Indrajeet yadav'
__version__ = '0.1'
__license__ = 'Nil'

import numpy as np
class parameters(object):

    def __init__(self, name_of_uav): 
        self.uav = name_of_uav 
        
    def mass(self):
        if self.uav == 'firefly': # wirefly without VI sensor: 1.53 and firefly with VI sensor 1.56779+0.105
        # also note that the vi_sensor put some mass offset on the vehicle so for now I added a constant torque in y direction 
            mass = 1.56779+0.100
 	    #mass = 1.792 # actual mass calculated in the lab
        elif self.uav == 'pelican': 
            #mass = 1.202 # As measured in the lab, simulator value is 1kg
	    mass = 1.0
        elif self.uav == 'hummingbird':
            #mass = 0.716 # asctec value
	    mass = 0.716 # measured in lab with most of electronics
        return mass

    def inertia(self):
        if self.uav == 'firefly': 
            inertia = np.array([[0.0347563,0,0],[0,0.0458929,0],[0,0,0.0977]])
            #inertia = np.array([[0.011,0,0],[0,0.0121,0],[0,0,0.02]]) # my experimental calcultions!!!!!!! why so much difference
        elif self.uav == 'pelican': 
            inertia = np.array([[0.01,0,0],[0,0.01,0],[0,0,0.02]]) # as measured in the lab, ETH calculated Iz as 0.02
        elif self.uav == 'hummingbird':
            inertia = np.array([[0.007,0,0],[0,0.007,0],[0,0,0.012]]) # asctec values
	    #inertia = np.array([[0.0036,0,0],[0,0.0036,0],[0,0,0.0065]]) # measured in lab, why half !!!
	    #inertia = np.array([[0.0045,0,0],[0,0.0045,0],[0,0,0.01]]) # new frame, approximate values just for testing
        return inertia

    def CG_to_CoR(self):
        if self.uav == 'firefly': 
            CG_to_CoR = 0.215
        elif self.uav == 'pelican': 
            CG_to_CoR = 0.21
        elif self.uav == 'hummingbird':
            CG_to_CoR = 0.17
        return CG_to_CoR      

    def kx(self):
        if self.uav == 'firefly': 
            kx = np.array([[6], [6], [6]]) # eth values 
	    #kx = np.array([[5], [5], [5]])
        elif self.uav == 'pelican': 
            kx = np.array([[4], [4], [4]])
	    #kx = np.array([[7], [7], [2]])
        elif self.uav == 'hummingbird':
            kx = np.array([[4], [4], [4]]) # z gain was 4
        return kx

    def kv(self):
        if self.uav == 'firefly': 
            kv = np.array([[5.7], [5.7], [5.7]])
            #kv = np.array([[3], [3], [3]])
        elif self.uav == 'pelican': 
            #kv = np.array([[2.7], [2.7], [2.7]]) # ETH values
	    kv = np.array([[4.7], [4.7], [4.7]])
        elif self.uav == 'hummingbird':
            kv = np.array([[2.2], [2.2], [2.2]]) # z gain was 2.2
        return kv

    def kR(self):
        if self.uav == 'firefly': 
            kR = np.array([[3], [3], [0.15]]) # ETH values
	    #kR = np.array([[1.0], [1.0], [0.05]])
        elif self.uav == 'pelican': 
            kR = np.array([[1], [1], [0.035]]) #ETH values
	    #kR = np.array([[0.5], [0.5], [0.15]])
        elif self.uav == 'hummingbird':
            kR = np.array([[0.7], [0.7], [0.035]])
        return kR

    def kOmega(self):
        if self.uav == 'firefly': 
            kOmega = np.array([[0.52], [0.52], [0.18]]) #values
	    #kOmega = np.array([[0.12], [0.12], [0.05]])
        elif self.uav == 'pelican': 
            kOmega = np.array([[0.22], [0.22], [0.01]]) # ETH values
	    #kOmega = np.array([[0.2], [0.2], [0.1]])
        elif self.uav == 'hummingbird':
            kOmega = np.array([[0.1], [0.1], [0.025]])
        return kOmega

    def tau_t(self):
        if self.uav == 'firefly': 
            tau_t = 8.54858e-6 # ETH values
	    #tau_t = 6.054858e-6 # my adjustment
        elif self.uav == 'pelican': 
            tau_t = 9.9865e-6 # eth value
	    #tau_t = 14.0e-6# calculated for non asctec propellers in the lab
        elif self.uav == 'hummingbird':
            tau_t = 8.04858e-06
	    #tau_t = 8.0e-06
	    #tau_t = 9.349e-06 # measured value by me, this was when battery was freshly charged
	    #tau_t = 7e-6 #my adjusted value to get the quad to hover at correct height, doesnt work 
        return tau_t       
    
    def tau_m(self):
        if self.uav == 'firefly': 
            tau_m = 1.6e-2
        elif self.uav == "pelican": 
            tau_m = 1.6e-2
        elif self.uav == 'hummingbird':
            tau_m = 1.6e-2
        return tau_m
    def gravity(self): 
        g = 9.8
        return g
    def identity(self): 
        I = np.array([[1,0,0],[0,1,0],[0,0,1]])
        return I

