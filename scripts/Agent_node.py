#!/usr/bin/env python


import rospy

from geometry_msgs.msg import Point, PoseStamped,TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from std_msgs.msg import Float32
#scipy /numpy
from scipy.spatial import distance
# OpenCV
import cv2

import math 
from math import *

import random
import numpy as np
import tf


######### TOPICS ###############
########################################################
SRV_ARM = '/mavros/cmd/arming'
SRV_SETMODE = '/mavros/set_mode'
SRV_TAKEOFF = '/mavros/cmd/takeoff'
TOPIC_SETPOINT_RAW = '/mavros/setpoint_raw/local'
TOPIC_SETPOINT = '/mavros/setpoint_position/local'
TOPIC_STATE = '/mavros/state'
TOPIC_LOCALPOSITION = '/mavros/local_position/pose'
TOPIC_SETVELOCITY = '/mavros/setpoint_velocity/cmd_vel'
TOPIC_MODELSTATE = '/gazebo/model_states'
TOPIC_SET_MODELSTATE =  '/gazebo/set_model_state'

TOPIC_SET_KSI =  '/ksi'
TOPIC_SET_BETA =  '/beta'
TOPIC_SET_ERROR =  '/error_dis'
TOPIC_SET_DIS =  '/dis'
TOPIC_EKF =  '/ekf'
TOPIC_GAMMA =  '/gamma'


TOPIC_SET_VX =  '/vx'
TOPIC_SET_VY =  '/vy'
TOPIC_SET_YAWRATE =  '/yawrate'

### Images
TOPIC_PUB = "/output/image_raw/compressed"
TOPIC_CAMINFO = "/camera_red_iris/camera_info"
TOPIC_IMGCOMPRESSED = "/camera_red_iris/image_raw/compressed"

########################################################
#rosbag record --duration=5m -e "(.*)(local_position/pose|ekf|ekf2|ekf3|ksi|beta|error_dis|dis|GazeboStates|vx|vy|yawrate)" 
#rosbag record -e -x "(.*)(/iris|)(.*)"

FREQ = 10
TS = 1/FREQ

VERBOSE=False


class Drone:


	def __init__(self,index = 1,name ='',rate =None ,pos_init =None):
		#################### Initialization ####################

		self.index = index
		self.name =  name
		self.pos_init = pos_init
		rospy.rate = rate
		self.img_node = None

		### Image parameters
		self.tag = "iris_"+str(index) # iris_0
		self.u = None
		self.v = None
		self.d = ''
		self.gamma =  [-0.0001]   # initial     
		self.img = None
		self.output_layers = None
		self.net  = None
		self.width  = 800
		self.detected = True # false detection

		### Pinhole Camera
		self.pinhole_camera_model = PinholeCameraModel()
		camera_info_topic = self.tag + TOPIC_CAMINFO
		camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)
		self.pinhole_camera_model.fromCameraInfo(camera_info) 

		#################### Setting the callBacks ####################
		self.position = PoseStamped()  # current position
		self.position_prev = None  # previous position
		self.positionDes = PositionTarget()

		#################### Subcriptions ####################
		self.sb_state = rospy.Subscriber(self.name + TOPIC_STATE,State,self.cb_state)
		self.sb_position = rospy.Subscriber(self.name + TOPIC_LOCALPOSITION,PoseStamped,self.cb_position)
		self.model_state = rospy.Subscriber( TOPIC_MODELSTATE,ModelStates,self.cb_model_state)
		self.sb_image = rospy.Subscriber(self.tag + TOPIC_IMGCOMPRESSED,CompressedImage, self.cb_image,  queue_size = 1) ## Images

		#################### Publications ####################
		self.sp_position = rospy.Publisher(self.name + TOPIC_SETPOINT_RAW, PositionTarget, queue_size=1) ## Setpoint
		self.sp_velocity = rospy.Publisher(self.name + TOPIC_SETVELOCITY, TwistStamped, queue_size=1) ##velocity
		self.image_pub = rospy.Publisher(self.tag + TOPIC_PUB,CompressedImage)                        ## Image

		#################### RosBag ####################
		self.sp_psi = rospy.Publisher(self.name + TOPIC_SET_KSI, Float32) # psi
		self.sp_beta = rospy.Publisher(self.name + TOPIC_SET_BETA, Float32) # beta
		self.sp_dis = rospy.Publisher(self.name + TOPIC_SET_DIS, Float32) # dis
		self.sp_error = rospy.Publisher(self.name + TOPIC_SET_ERROR, Float32) # error distance
		self.sp_gamma = rospy.Publisher(self.name + TOPIC_GAMMA, Float32) # gamma

		self.sp_vx = rospy.Publisher(self.name + TOPIC_SET_VX, Float32) # VX
		self.sp_vy = rospy.Publisher(self.name + TOPIC_SET_VY, Float32) # VY
		self.sp_yawrate = rospy.Publisher(self.name + TOPIC_SET_YAWRATE, Float32) # yaw

		self.pub_gazebo = rospy.Publisher( 'GazeboStates', ModelStates, queue_size=1)


	######### Publishers ###############
	########################################################

	# Publisher position -- Param PositionTarget
	def setPoint(self,x,y,z=3.0,yaw=0.):
		position = self.generatePosition(x ,y,z,yaw)
		self.sp_position.publish(position)

	# Publisher velocity
	def setVelocity(self,pub):
		self.sp_velocity.publish(pub)


	# Generate a target position
	# The output is a PositionTarget
	def generatePosition(self,x,y,z=3.0,yaw=0.):
		''' instantiate setpoint msg '''
		sp = PositionTarget()
		sp.velocity.x =  x
		sp.velocity.y =  y
		#sp.velocity.z = 0.
		sp.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ  + PositionTarget.IGNORE_YAW  + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY  
		# sp.yaw =  yaw     # the yaw value
		sp.yaw_rate = yaw	# the yaw velocity	 																																																	
		sp.coordinate_frame = 8
		sp.position.z = z
		return sp

	# Publish for rosbag
	def set_psi_beta(self,psi,beta,dis,error,gamma):
		self.sp_psi.publish(psi)
		self.sp_beta.publish(beta)
		self.sp_dis.publish(dis)
		self.sp_error.publish(error)
		self.sp_gamma.publish(gamma)

	def set_velocities(self,vx,vy,yawrate):
		self.sp_vx.publish(vx)
		self.sp_vy.publish(vy)
		self.sp_yawrate.publish(yawrate)

	######### Callbacks ###############
	########################################################

	# callBack gazebo Model states
	def cb_model_state(self,state):
		self.model_state = state

	# Publish RosBags Gazebo States
	def pub_gazeboStates(self,data):
		self.pub_gazebo.publish(data)


	# callback for mavros the position
	def cb_position(self,position):
		if self.position_prev == None :
			self.position_prev = position
		else :
			self.position_prev = self.position
		self.position = position


	# callBack mavros State
	def cb_state(self,state):
		self.state = state


	def startDetection(self):
		self.img_node  = Image_node(x,self.rate)



	#########################################################
	## callback for camera Images
	#########################################################
	def cb_image(self, ros_data):

		''' Here images get converted and features detected'''
		# rospy.loginfo("start time %s %i %i ",self.tag, now.secs, now.nsecs)        
		if VERBOSE :
		    print ('received image of type: "%s"' % ros_data.format)

		#### direct conversion to CV2 ####
		np_arr = np.fromstring(ros_data.data, np.uint8) # image as numpy array
		image_np = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR) 
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
		self.img =  image_np # assign image 		


	######### Image processing with YOLO #########################
	##############################################################
	def process_img(self):
		names = 'uav.names'

		CONF_THRESH, NMS_THRESH = 0.25, 0.25  # threshold values
		############# Deep Larning Processing ###################
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
		    # When there is new detection 
		    self.gamma = [] # clear Previous gammas
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
		        self.gamma.append(self.u - width/2)

		        # if self.detected == True :
		        #     self.gamma = self.u - width/2

		return self.gamma




	######### Engine Configurations Services ###############
	########################################################
	def fly(self):
		''' Hovering the agent '''
		self.setPoint(self.pos_init[0],self.pos_init[1],self.pos_init[2])


	def setArm(self):
		''' Arming '''
		print (self.name)
		srv = self.name + SRV_ARM
		rospy.wait_for_service(srv)
		try:
			srv_arm = rospy.ServiceProxy(srv,mavros_msgs.srv.CommandBool)
			srv_arm(True)
			print("is armed")
		except rospy.ServiceException,e :
			print("Service arming call failed: %s"%e)


	def setDisarm(self):
		''' Disarming '''
		srv = self.name + SRV_ARM
		rospy.wait_for_service(srv)
		try :
			srv_disarm = rospy.ServiceProxy(srv,mavros_msgs.srv.CommandBool)
			srv_disarm(False)
		except  rospy.ServiceException,e:
			print("Service disarming call failed: %s"%e)


	def setTakeoff(self):
		''' Taking off '''
		srv = self.name + SRV_TAKEOFF
		rospy.wait_for_service(srv)
		try :
			srv_takeoff = rospy.ServiceProxy(srv,mavros_msgs.srv.CommandTOL)
			srv_takeoff(altitude=3)
		except  rospy.ServiceException,e:
			print("Service disarming call failed: %s"%e)			


	def setMode(self,mode ='OFFBOARD'):
		''' Changing Mode '''
		srv = self.name + SRV_SETMODE
		rospy.wait_for_service(srv)
		try :
			srv_setmode = rospy.ServiceProxy(srv,mavros_msgs.srv.SetMode)
			srv_setmode(custom_mode=mode)
		except  rospy.ServiceException,e:
			print("Service offboard call failed: %s"%e)