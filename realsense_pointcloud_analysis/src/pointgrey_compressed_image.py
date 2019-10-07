#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 14:58:59 2019
File to read octomap msgs
@author: ind
"""
import numpy as np 
import rospy, cv2
import matplotlib.pyplot as plt
from matplotlib import cm
from operator import mul, add
from numpy.random import uniform as uni
from sensor_msgs.msg import CompressedImage, Image

class pointgrey_compressed_image(object):
    def __init__(self): 
        """initializes various paramters"""
        self.counter = 0
        open("freecell_data.txt", "w").close()
        try: 
            rospy.Subscriber("/camera1/image_color", Image, self.left_image_callback, tcp_nodelay = False)
            rospy.Subscriber("/camera2/image_color", Image, self.right_image_callback, tcp_nodelay = False)
        except: 
            print 'problem subcribing to one of the image topics'

        
    def left_image_callback(self, data):
        """compress the images from pointgreycamera"""
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        msg = CompressedImage()
        msg.header.stamp = data.header.stamp
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        





if __name__ == '__main__':
    rospy.init_node('pointgrey_compressed_image_node', anonymous=True, log_level=rospy.DEBUG)
    r = rospy.Rate(20)    
    traj = pointgrey_compressed_image()
    try:
        while not rospy.is_shutdown():             
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass
