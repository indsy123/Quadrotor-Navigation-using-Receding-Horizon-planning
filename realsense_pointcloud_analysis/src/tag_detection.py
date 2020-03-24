#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 14:58:59 2019
script to detect tags
@author: ind
"""
import numpy as np 
import rospy, cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from matplotlib import cm
from operator import mul, add
import ros_numpy, time
from numpy.random import uniform as uni
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped

class tag_detection(object):
    def __init__(self): 
        """initializes various paramters"""
        self.counter = 0
        self.camera_matrix_d435 = np.array([[609.917, 0, 322.003], [0, 609.755, 236.385], [0, 0, 1]])
        self.dist_coeffs_d435 = np.array([0, 0, 0, 0])
        self.camera_matrix_t265 = np.array([[285.5, 0.0, 423.362], [0.0, 285.6, 394.64], [0, 0, 1]])
        self.dist_coeffs_t265 = np.array([-0.007989956066012383, 0.04564497992396355, -0.04361240938305855, 0.0083484360948205])
        self.target_publish = rospy.Publisher("/udrone1/target", PoseStamped, queue_size = 1, tcp_nodelay = False)
        try: 
            rospy.Subscriber("/udrone1/d435/color/image_raw", Image, self.image_callback, tcp_nodelay = False)
            #rospy.Subscriber("/udrone1/t265/fisheye1/image_raw", Image, self.image_callback, tcp_nodelay = False)

        except: 
            print 'problem subcribing to one of the image topics'

        
    def image_callback(self, data):
        """compress the images from pointgreycamera"""
        start = time.time()
        image_array = ros_numpy.numpify(data)
        print '1', time.time()-start
        image_array_gray = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
        print '2', time.time()-start
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        print '3', time.time()-start
        parameters =  aruco.DetectorParameters_create()
        print '4', time.time()-start
        corners, ids, rejectedImgPoints = aruco.detectMarkers(image_array_gray, aruco_dict, parameters=parameters)
        print '5', time.time()-start
        
        
        markerLength = 0.176
        rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, self.camera_matrix_d435, self.dist_coeffs_d435) # For a single marker
        print '6', time.time()-start        
        try:         
            msg = PoseStamped()
            msg.header.stamp = data.header.stamp
            msg.pose.position.x = tvec[0][0][0]; 
            msg.pose.position.y = tvec[0][0][1]; 
            msg.pose.position.z = tvec[0][0][2] 
            self.target_publish.publish(msg)
            print 'translation and rotations are of the marker are:', tvec, rvec
        except: 
            print 'Aruco Tag occuded, no target can be tracked'
	"""
        frame_markers = aruco.drawDetectedMarkers(image_array.copy(), corners, ids)
        plt.imshow(frame_markers)
        for i in range(len(ids)):
            c = corners[i][0]
            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
        plt.legend()
        plt.pause(0.001) 
	"""





if __name__ == '__main__':
    rospy.init_node('tag_detection', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(20)    
    traj = tag_detection()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
