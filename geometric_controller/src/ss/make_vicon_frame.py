# -*- coding: utf-8 -*-
"""
Created on Wed Jan 17 10:21:21 2018

@author: indrajeet
"""

import numpy as np 
import matplotlib.pyplot as plt
import math 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from pyquaternion import Quaternion 
import open3d
import multiprocessing

class rotation(object):
    def __init__(self): 
        # enter each marker position 
        # this was for pelican marker2 is top marker exactly(hopefully:)) above cog 
        self.unlabelled1 = np.array([[0.00838234329224], [-0.000316759377718], [0.249092102051]])        
        self.unlabelled4 = np.array([[0.1220337677], [0.00299229168892], [0.231751296997]])
        self.unlabelled0 = np.array([[-0.00916078948975], [0.122241714478], [0.196119125366]])
        self.unlabelled2 = np.array([[0.0211480846405], [-0.11208506012], [0.195752105713]])
        self.unlabelled3 = np.array([[-0.120279327393], [-0.025981470108], [0.229515701294]])
        #self.unlabelled4 = np.array([[-0.0806915512085], [-0.0866674880981], [0.158799499512]])      
     
        # put the center and orientation of the initial frame: 
        #self.location_of_cog = np.array([[0], [0], [0.191985595703-0.07]]) #diy2
        self.location_of_cog = np.array([[0.00838234329224], [-0.000316759377718], [0.249092102051-0.11]]) #diy2




    def rotate_points(self):
            

        # vector from marker unlabeled0 to all others 
        m0 = self.unlabelled0-self.location_of_cog
        m1 = self.unlabelled1-self.location_of_cog
        m2 = self.unlabelled2-self.location_of_cog
        m3 = self.unlabelled3-self.location_of_cog
        m4 = self.unlabelled4-self.location_of_cog
        #m5 = self.unlabelled5-self.location_of_cog
        #m6 = self.unlabelled6-self.location_of_cog
        #m7 = self.unlabelled7-self.location_of_cog
        #m8 = self.unlabelled8-self.location_of_cog
        print 'm1:', 1000*m1, 'm4:',1000*m4, 'm0:', 1000*m0, 'm2:', 1000*m2 ,'m3:', 1000*m3# , 'm4:', 1000*m4#, 'm4:', 1000*m4 #, 'm1:', 1000*m1 

if __name__ == '__main__':
    f = rotation()
    f.rotate_points()
