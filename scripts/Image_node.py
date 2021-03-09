#!/usr/bin/env python



import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import math


# OpenCV
import cv2


# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import warnings
warnings.filterwarnings("ignore")
import time


import argparse

VERBOSE=False

TOPIC_PUB = "/output/image_raw/compressed"
TOPIC_CAMINFO = "/camera_red_iris/camera_info"
TOPIC_IMGCOMPRESSED = "/camera_red_iris/image_raw/compressed"

# weights = '../../yolov3uav_3305.weights'
# config = 'yolov3uav.cfg'
# names = 'uav.names'

class Image_node:

    def __init__(self,tg,rate):
        self.tag = "iris_"+str(tg) # iris_0
        self.tg = tg
        rospy.rate = 1000
        self.u = None
        self.v = None
        self.d = ''
        self.gamma =  -0.0001        
        self.img = None
        self.output_layers = None
        self.net  = None
        self.width  = 800

        self.detected = True # should update the image in case of false
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher(self.tag + TOPIC_PUB,
            CompressedImage)

        # self.bridge = CvBridge()

        self.pinhole_camera_model = PinholeCameraModel()
        camera_info_topic = self.tag + TOPIC_CAMINFO
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)
        self.pinhole_camera_model.fromCameraInfo(camera_info) 

        # subscribed Topic
        self.subscriber = rospy.Subscriber(self.tag + TOPIC_IMGCOMPRESSED,
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

    def callback(self, ros_data):

        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''

        # rospy.loginfo("start time %s %i %i ",self.tag, now.secs, now.nsecs)        
        if VERBOSE :
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8) # image as numpy array
        image_np = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR) 
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        self.img =  image_np # assign image 

    def process_img(self):
        names = 'uav.names'
        CONF_THRESH, NMS_THRESH = 0.25, 0.25  # threshold values
        ############# Deep Larning Processing ##############
        # Load the network 
        net = cv2.dnn.readNet('yolo/yolov3-tiny.cfg', 'yolo/yolov3tiny_2000.weights')
        # net = cv2.dnn.readNet('yolov3uav.cfg', '../../yolov3uav_2000.weights')
        # net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV) # using Cuda/Cpu
        # net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU) # using Cuda/Cpu

        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA) # using Cuda/gpu
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA) # using Cuda/gpu

        # Get the output layer from YOLO
        layers = net.getLayerNames()
        output_layers = [layers[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        # Read and convert the image to blob and perform forward pass to get the bounding boxes with their confidence scores
        height, width = self.img.shape[:2]
        blob = cv2.dnn.blobFromImage(self.img, 0.00392, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)

        layer_outputs = net.forward(output_layers)
        #############  BoundingBox Processing ##############
        class_ids, confidences, b_boxes = [], [], []
        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > CONF_THRESH:
                    center_x, center_y, w, h = (detection[0:4] * np.array([width, height, width, height])).astype('int')

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    b_boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(confidence))
                    class_ids.append(int(class_id))


        ''' Perform non maximum suppression for the 
        bounding boxes to filter overlapping and low 
        confident bounding boxes '''

        indices = cv2.dnn.NMSBoxes(b_boxes, confidences, CONF_THRESH, NMS_THRESH)
        if len(indices) > 0 :
            indices = cv2.dnn.NMSBoxes(b_boxes, confidences, CONF_THRESH, NMS_THRESH).flatten().tolist()
            # Draw the filtered bounding boxes with their class to the image
            with open(names, "r") as f:
                classes = [line.strip() for line in f.readlines()]
            colors = np.random.uniform(0, 255, size=(3, 3))
            # if len(indices) > 1 : # multiple object detection
            for index in indices:
                x, y, w, h = b_boxes[index]
                self.detected = True # 
                self.u = x + w / 2 
                self.v =  y + h /2 

                if self.detected == True :
                    self.gamma = self.u - width/2

        return self.gamma


        # self.dect_count += 1
        # print(str(self.tg),'detec',self.dect_count)

        # #### Create CompressedIamge ####
        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        # # cv2.imwrite('images/'+str(self.tg) + '/' + str(rospy.Time.now())+ 'Imgdetect.jpg', img)
        # cv2.imwrite('images/'+str(self.tg) + '/' + 'Imgdetect.jpg', img)

        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', self.img)[1]).tostring()
        # # Publish new image
        # self.image_pub.publish(msg)

        # print('taggg ' ,self.tag)
        # now = rospy.get_rostime()
        # diff  = start - now.nsecs
        # print( str(self.tg)+ 'difference in detection ' , (diff/1000000) )


        # rospy.loginfo("Finish  time %s %i %i ",self.tag, now.secs, now.nsecs) 
        # rosrun image_view image_view image:=/iris_2/output/image_raw  ~image_transport:=compressed
        # rosrun image_view image_view image:=/iris_2/output/image_rawaw/compressed  ~image_transport