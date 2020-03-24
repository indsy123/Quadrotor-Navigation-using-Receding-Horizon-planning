#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
original script created by Michael. The script detects a quad, puts  
a bounding box around it and also returns detection confidence. 
It is under active modification to convert it into a ros node and enhance the 
speed to get a real time performance. The aim is to track a quad from the 
other quad. 
@author: Michael Seebok
@Modified by: Indrajeet Yadav
Modifications in R0: changed to class structure
Modification in R1: change to take data from a ros bag
"""

import sys
import os
import cv2, time
import numpy as np
import tensorflow as tf
#import ros_numpy

#os.environ['CUDA_VISIBLE_DEVICES'] = '0'


sys.path.append("..")

from object_detection.utils import label_map_util
from matplotlib import pyplot as plt
from utils import visualization_utils as vis_util
from tensorflow.python.compiler.tensorrt import trt_convert as trt


class quadrotor_detection_class(object):
    def __init__(self, name, f, base, ckpt_path, labels_path, n_classes): 
        """initializes various paramters"""
        self.start = time.time()
        self.logpath = '/home/pelican/data_collection/'
        open(self.logpath+'inference_confidence.txt', 'w').close()

        self.MODEL_NAME = name
        self.MODEL_FILE = f
        #self.DOWNLOAD_BASE = base
        #PATH_TO_CKPT = os.path.join( 'fine_tuned_model','frozen_inference_graph.pb')
        #PATH_TO_LABELS = os.path.join('data', 'pascal_label_map.pbtxt')
        self.PATH_TO_CKPT = ckpt_path
        self.PATH_TO_LABELS = labels_path
        self.NUM_CLASSES = n_classes
        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR) # after first attemptipn is doesnt print all messages, tried to save time

        """
        #with tf.Session as sess: 
        # First deserialize your frozen graph:
        with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as f:
            frozen_graph = tf.GraphDef()
            frozen_graph.ParseFromString(f.read())
            tf.import_graph_def(frozen_graph, name='')
        # Now you can create a TensorRT inference graph from your
        # frozen graph:
        converter = trt.TrtGraphConverter(input_graph_def=frozen_graph, nodes_blacklist=['logits', 'classes']) #output nodes
        trt_graph = converter.convert()
        # Import the TensorRT graph into a new graph and run:
        self.detection_graph = tf.import_graph_def(trt_graph, return_elements=['logits', 'classes'])
        #sess.run(self.detection_graph)
        """
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
        
        # running session only at the start, see what happens or change it back to what Michael has  
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        #config.log_device_placement=True
        self.session = tf.Session(graph=self.detection_graph, config=config)


    def detect_alert(self, boxes, classes, scores, category_index, max_boxes_to_draw=20, min_score_thresh=.5):
        r = []
        for i in range(min(max_boxes_to_draw, boxes.shape[0])):
            if scores is None or scores[i] > min_score_thresh:
                test1 = None
                test2 = None
    
                if category_index[classes[i]]['name']:
                    test1 = category_index[classes[i]]['name']
                    test2 = int(100 * scores[i])
    
                line = {}
                line[test1] = test2
                r.append(line)
    
        return r
    
    def detect_objects(self, image_np, sess, detection_graph, category_index):
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        start = time.time()
        # Actual detection.
        (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections], feed_dict={image_tensor: image_np_expanded})
        print 'time taken in actual detection', time.time()-start
        alert_array = self.detect_alert(np.squeeze(boxes), np.squeeze(classes).astype(np.int32), np.squeeze(scores), category_index)   
        return alert_array, (boxes, scores, classes, num_detections)



    def load_image_into_numpy_array(self, image):
        """not needed for now, remove it and if needed use ros_numpy.numpify(image) 
        because eventually you would be taking images from ros topics"""
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)



    def process_image_and_plot(self, detection_graph, image, category_index):
        
        with tf.device('/cpu:0'):
            with detection_graph.as_default():
                #with tf.Session(graph=detection_graph, config=tf.ConfigProto(log_device_placement=False)) as sess:
                #with self.session as sess:
                
                alert_array, (boxes, scores, classes, num_detections) = self.detect_objects(image, self.session, detection_graph, category_index)
                box_coordinates = boxes[0][0]
                #print 'box pixel coordinates xmin, ymin, xmax, ymax:', box_coordinates[0]*1080, box_coordinates[1]*1920, box_coordinates[2]*1080, box_coordinates[3]*1920
                # wont be needing visualization, just return the confidence level
                print alert_array
                #f = open(self.logpath+'inference_confidence.txt', 'a')
                #f.write('%s\n' %(alert_array))
                
                try:                     
                    if alert_array[0]['quad'] > 75: 
                        vis_util.visualize_boxes_and_labels_on_image_array(
                              image,
                              np.squeeze(boxes),
                              np.squeeze(classes).astype(np.int32),
                              np.squeeze(scores),
                              category_index,
                              use_normalized_coordinates=True,
                              line_thickness=4)
                        #plt.figure(figsize=(12, 8))
                        #plt.imshow(image)
                        cv2.imshow("tagged_image", image)
                        cv2.waitKey(1)
                        
                        #plt.pause(0.00001)
                        #plt.show()
                    else: 
                        print 'detection confdence is less than 75%'
                except: 
                    print 'alert_array is null'
                

    def detect_quad(self): 

        label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)

    
        """
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        """        
        
        video = cv2.VideoCapture('/home/pelican/catkin_ws/src/realsense_pointcloud_analysis/src/detection_gore_hall.mp4') #change the path of the video
        #success, image = video.read()
        count = 0
        success = True
        while success:
            success, image = video.read()
            #print('Read a new frame: ', success)
            if success:
                start  = time.time()
                #img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                self.process_image_and_plot(self.detection_graph, image, category_index)
                print ' total time taken to execute the script is:', '2', time.time()-start
                #cv2.imshow('image',img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                #if count == 200:
                #    alert = process_image(img)
                #    count = 0
                   # if alert:
                        #break
            count += 1
        
        video.release()
        """

        image = cv2.imread('/home/ind/data_collection/image_for_RCNN/frame0016.jpg') #change the path of the video
        #image = cv2.resize(image, (200, 200))
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.process_image_and_plot(self.detection_graph, img, category_index)
        """
        print ' total time taken to execute is:', time.time()-self.start


        #cv2.destroyAllWindows()



        


if __name__ == '__main__':
    # all this may eventually come from a script which calls this script
    MODEL_NAME = 'mask_rcnn_inception_v2_coco_2018_01_28'
    MODEL_FILE = MODEL_NAME + '.tar.gz'
    DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/' # why do I need that?? Ask Michael 
    PATH_TO_CKPT = '/home/pelican/catkin_ws/src/realsense_pointcloud_analysis/src/fine_tuned_model/frozen_inference_graph.pb'
    PATH_TO_LABELS = '/home/pelican/catkin_ws/src/realsense_pointcloud_analysis/src/data/pascal_label_map.pbtxt'
    NUM_CLASSES = 1 # at some point get rid of it, it is not needed
    solve = quadrotor_detection_class(MODEL_NAME, MODEL_FILE, DOWNLOAD_BASE, PATH_TO_CKPT, PATH_TO_LABELS, NUM_CLASSES)
    solve.detect_quad()

