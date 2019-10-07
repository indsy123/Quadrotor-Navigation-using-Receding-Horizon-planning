#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 14:58:59 2019
File to read octomap msgs
@author: ind
"""
import numpy as np 
import rospy 
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from operator import mul, add
from numpy.random import uniform as uni
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import MarkerArray
import scipy.interpolate
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import networkx

class read_octomap(object):
    def __init__(self): 
        """initializes various paramters"""
        self.counter = 0
        open("freecell_data.txt", "w").close()
        try: 
            rospy.Subscriber('/free_cells_vis_array', MarkerArray, self.callback, queue_size = 1, tcp_nodelay = True)
            #rospy.Subscriber('/octomap_binary', Octomap, self.callback, queue_size = 1, tcp_nodelay = True)
        except: 
            print 'problem subcribing to octomap'

        
    def callback(self, data):
        """
        a = time.time()
        p = [[data.markers[-1].points[j].x, data.markers[-1].points[j].y, data.markers[-1].points[j].z] for j in range(len(data.markers[-1].points))]
        p = zip(*p)
        #print 'time taken is:', time.time() - a
        
        fig = plt.figure(figsize=(8,8))
        ax = fig.gca(projection='3d')
        ax.scatter(p[0], p[1], p[2], color='r')           
        plt.ion()
        plt.show()

        """
        
        if self.counter == 0:
            a = time.time()
            f1 = open('freecell_data.txt','a')
            f1.write('%s\n' %(data))
            color = ['r', 'g', 'b', 'm', 'y', 'c', 'k']
            fig = plt.figure(figsize=(8,8))
            ax = fig.add_subplot(111, projection='3d')
            for n in range(len(data.markers)): 
                if data.markers[n].action == 0: 
                    p = [[data.markers[n].points[j].x, data.markers[n].points[j].y, data.markers[n].points[j].z] for j in range(len(data.markers[n].points))]
                    p = zip(*p)
                    #print len(p[0]), time.time()-a  
 
                    #ax = fig.gca(projection='3d')
                    ax.set_xlim(0, 4); ax.set_ylim(-2, 2); ax.set_zlim(-2,2)
                    ax.set_xlabel('X Label'); ax.set_ylabel('Y Label'); ax.set_zlabel('Z Label')
                    ax.scatter(p[0], p[1], p[2], marker = 'o', c=color[len(data.markers)-n])  
                    #plt.show()
            #marker_actions.append([1 if data.markers[n].action == 0 else 0] for n in range(len(data.markers)))
            #print marker_actions
            
            #print 'len of the data.markera is:', len(data.markers)
            #p = [[data.markers[14].points[j].x, data.markers[14].points[j].y, data.markers[14].points[j].z] for j in range(len(data.markers[14].points))]
            #p = zip(*p)
            ##print 'time taken is:', time.time()-a            
            #fig = plt.figure(figsize=(8,8))
            #ax = fig.add_subplot(111, projection='3d')
            ##ax = fig.gca(projection='3d')
            #ax.set_zlim(-2, 2); ax.set_ylim(-2, 2); ax.set_xlim(0,2.5)
            #ax.set_xlabel('X Label'); ax.set_ylabel('Y Label'); ax.set_zlabel('Z Label')
            #ax.scatter(p[0], p[1], p[2], marker = 'o', c='r')   
            
            #plt.show()

            #p = [[data.markers[14].points[j].x, data.markers[14].points[j].y, data.markers[14].points[j].z] for j in range(len(data.markers[14].points))]
            #p = zip(*p)
            #print 'time taken is:', time.time()-a            
            #fig = plt.figure(figsize=(8,8))
            #ax = fig.gca(projection='3d')            
            #ax.scatter(p[0], p[1], p[2], marker = 'o', c='g')  
            #ax.set_xlim(-2, 2); ax.set_ylim(-2, 2); ax.set_zlim(0,2.5)
            #ax.set_xlabel('X Label'); ax.set_ylabel('Y Label'); ax.set_zlabel('Z Label')
            #plt.show()

            #p = [[data.markers[15].points[j].x, data.markers[15].points[j].y, data.markers[15].points[j].z] for j in range(len(data.markers[15].points))]
            #p = zip(*p)
            #print 'time taken is:', time.time()-a            
            #fig = plt.figure(figsize=(8,8))
            #ax = fig.gca(projection='3d')
            #ax.scatter(p[0], p[1], p[2], marker = 'o', c='b')  
            #ax.set_xlim(0, 2.5); ax.set_ylim(-2, 2); ax.set_zlim(-2,2)
            #ax.set_xlabel('X Label'); ax.set_ylabel('Y Label'); ax.set_zlabel('Z Label')
            #plt.show()

            #p = [[data.markers[16].points[j].x, data.markers[16].points[j].y, data.markers[16].points[j].z] for j in range(len(data.markers[16].points))]
            #p = zip(*p)
            #print 'time taken is:', time.time()-a            
            #fig = plt.figure(figsize=(8,8))
            #ax = fig.gca(projection='3d')
            #ax.scatter(p[0], p[1], p[2], marker = 'o', c='m') 
            ##ax.voxel(p[0], p[1], p[2], facecolors='k')
            #ax.set_xlim(0, 2.5); ax.set_ylim(-2, 2); ax.set_zlim(-2,2)
            #ax.set_xlabel('X Label'); ax.set_ylabel('Y Label'); ax.set_zlabel('Z Label')
            plt.show()

            
        else:
            print 'done writing first dataset'
            pass
        self.counter += 1




if __name__ == '__main__':
    #name = rospy.get_param('uav_name')
    #freq = rospy.get_param('publish_frequency')
    rospy.init_node('read_octomap', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(1)    
    traj = read_octomap()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
