#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
The script is a ros node that generates the pointcloud from the t265 camera. 
The ros stack by realsense gives pointcloud for D415, D435 cameras but not for
t265 tracking visensor. Many parts of the ndoes are taken from the file 
"t265_stereo.py" in librealsense directory and used with some modifications. 
Comments have been added to various places. 
Author: ISY
Date: 18th Sept 2019
"""

#import matplotlib.pyplot as plt
import numpy as np
from math import tan, pi
import rospy
import cv2, time
import message_filters
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
#from mpl_toolkits.mplot3d import Axes3D
import std_msgs.msg

class pointcloud_from_t265(object):
    def __init__(self): 
        """initialization"""
        self.counter = 1
        self.baseline = 0.0637 # baseline of the t265
        # hard coded for now, take it from lauch or calibration file after the code work.
        # Distortion Coefficients
        self.D0 = np.array([-0.0848522710825053, 0.25362120938018384, -0.2551190188579361, 0.08022766563879905])
        self.D1 = np.array([0.041634397718402064, -0.07541171196161034, 0.06973719488574245, -0.026937810584564664])
        # Intrinsics
        self.P0 = np.array([[285.5000915527344, 0.0, 423.3619079589844, 0.0], \
                            [0.0, 285.5920104980469, 394.6387939453125, 0.0], \
                            [0.0, 0.0, 1.0, 0.0]])
        self.P1 = np.array([[284.9872131347656, 0.0, 422.3194885253906, 0.0], \
                            [0.0, 284.96148681640625, 399.1438903808594, 0.0], \
                            [0.0, 0.0, 1.0, 0.0]])
        extrinsics = np.array([[ 0.99997339,  0.00323771, -0.00653784, -0.06369426], \
                                [-0.00321873,  0.99999058,  0.0029125,  -0.00036734], \
                                [ 0.00654721, -0.00289138,  0.99997439, -0.00046723], \
                                [ 0.0,          0.0,          0.0,          1.0    ]])

        self.win_size = 5
        self.min_disp = 0
        self.max_disp = 112
        self.num_disp = self.max_disp - self.min_disp
        
        # taken from librealsense
        # We calculate the undistorted focal length:
        #
        #         h
        # -----------------
        #  \      |      /
        #    \    | f  /
        #     \   |   /
        #      \ fov /
        #        \|/
        stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
        stereo_height_px = 300          # 600x600 pixel stereo output, changed to 600 from 300, lets see what happens
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)
    
        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras taken from calibre
        self.R_left = np.eye(3)
        self.R_right = extrinsics[0:3, 0:3]
        T = extrinsics[0:3, 3:4]
    
        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired output region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        stereo_width_px = stereo_height_px + self.max_disp
        self.stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + self.max_disp
        stereo_cy = (stereo_height_px - 1)/2
    
        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length
    
        self.K0 = self.P0[0:3, 0:3]; self.K1 = self.P1[0:3, 0:3]
        self.P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                           [0, stereo_focal_px, stereo_cy, 0],
                           [0,               0,         1, 0]])
        self.P_right = self.P_left.copy()
        self.P_right[0][3] = T[0][0]*stereo_focal_px
    
        # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
        # since we will crop the disparity later
        self.Q = np.array([[1, 0,       0, -(stereo_cx - self.max_disp)],\
                      [0, 1,       0, -stereo_cy],\
                      [0, 0,       0, stereo_focal_px],\
                      [0, 0, -1/T[0][0], 0]])
                      
        #self.Q = np.array([[1, 0,       0, -423.0],\
        #              [0, 1,       0, -395.0],\
        #              [0, 0,       0, 285.0],\
        #              [0, 0, -1/T[0][0], 0]])
                      
                      
        #print stereo_cx, stereo_cy, T[0][0], self.Q
                  
        self.pcpub = rospy.Publisher('/camera/t265/points', PointCloud2, queue_size = 1, tcp_nodelay = True)
        
        try: 
            image_left = message_filters.Subscriber("/camera/fisheye1/image_raw/compressed", CompressedImage, tcp_nodelay = True)
            image_right = message_filters.Subscriber("/camera/fisheye2/image_raw/compressed", CompressedImage, tcp_nodelay = True)
            ts = message_filters.TimeSynchronizer([image_left, image_right], 10)
            ts.registerCallback(self.callback)
            #rospy.loginfo("Subscribing to: " + self.sub.resolved_name + " of type: " + str(self.sub.type))
        except:
            print('problem subscribing to one of the camera topic')


    def callback(self,  image_left, image_right):
        """creates point cloud from stereo camera"""
        a = time.time()
        #print a
        array1 = np.fromstring(image_left.data, dtype=np.uint8)

        imgL = cv2.imdecode(array1,cv2.IMREAD_GRAYSCALE)
        array2 = np.fromstring(image_right.data, dtype=np.uint8)
        imgR = cv2.imdecode(array2, cv2.IMREAD_GRAYSCALE) 

        #print '1', time.time()-a
        
        stereo = cv2.StereoSGBM_create(
                                        minDisparity = self.min_disp,
                                        numDisparities = self.num_disp,
                                        blockSize = 16,
                                        uniquenessRatio = 10,
                                        speckleWindowSize = 100,
                                        speckleRange = 32,
                                        disp12MaxDiff = 1,
                                        P1 = 8*3*self.win_size**2,
                                        P2 = 32*3*self.win_size**2,
                                        #mode = StereoSGBM::MODE_HH,
                                        )

        """
        stereo = cv2.StereoBM_create(numDisparities = self.num_disp, blockSize = 15)
        stereo.setPreFilterSize(31)#41
        stereo.setPreFilterType(cv2.STEREO_BM_PREFILTER_NORMALIZED_RESPONSE)
        stereo.setPreFilterCap(31)
        stereo.setTextureThreshold(10)
        stereo.setMinDisparity(self.min_disp)
        stereo.setSpeckleWindowSize(100)
        stereo.setSpeckleRange(32)
        stereo.setUniquenessRatio(10)
        """

        #print '2', time.time()-a
        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(self.K0, self.D0, self.R_left, self.P_left, self.stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(self.K1, self.D1, self.R_right, self.P_right, self.stereo_size, m1type)

        #print '3', time.time()-a
        #print 'lm, rm size', lm1.shape, lm2.shape, rm1.shape, rm2.shape, 'img size', imgL.shape, imgR.shape

        #dst1 = np.zeros(array1.shape[0])

        center_undistorted_left = cv2.remap(imgL, lm1, lm2, interpolation = cv2.INTER_LINEAR)
        center_undistorted_right = cv2.remap(imgR, rm1, rm2, interpolation = cv2.INTER_LINEAR)
        print '4', time.time()-a
        # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
        disparity = stereo.compute(center_undistorted_left, center_undistorted_right).astype(np.float32) / 16.0

        # re-crop just the valid part of the disparity
        disparity = disparity[:,self.max_disp:]

        #print '5', time.time()-a
        #a, b = disparity.shape
        #if self.counter == 1: 
        #    for j in range(a): 
        #        f1 = open('disparty', 'a')
        #        f1.write('%s\n' %(disparity[j]))
        #true_map = disparity.astype(np.float32) / 16.0



        points = cv2.reprojectImageTo3D(disparity, self.Q)
        #print '6', time.time()-a

        mask = disparity > disparity.min()
        
        out_points = points[mask]
        #out_colors = colors[mask]
        
        #print '7', time.time()-a
        # publish generated pointcloud
        
        #out_points = out_points[~np.isinf(out_points).any(axis=1)]

        header = std_msgs.msg.Header()
        header.stamp = image_left.header.stamp #rospy.Time.now()
        header.frame_id = 'world'
        p = pc2.create_cloud_xyz32(header, out_points)
        self.pcpub.publish(p)  
        print '8', time.time()-a

        self.counter = self.counter + 1

        
        

if __name__ == '__main__':
    rospy.init_node('stereo_to_pointcloud', anonymous=False, log_level=rospy.DEBUG)
    r = rospy.Rate(20)
    c = pointcloud_from_t265()
    try: 
        while not rospy.is_shutdown(): 
            rospy.spin()
    except rospy.ROSInterruptException(): 
        pass

