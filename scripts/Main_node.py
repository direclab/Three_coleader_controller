#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, PoseStamped,TwistStamped, Twist
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.msg import *

#scipy /numpy
from scipy.spatial import distance

import math 
from math import *

import random
import numpy as np
import tf

# Ros camera
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

##########

from Image_node import *
from Agent_node  import *
import time

#####
#Topics
# SRV_ARM = '/mavros/cmd/arming'
# SRV_SETMODE = '/mavros/set_mode'
# SRV_TAKEOFF = '/mavros/cmd/takeoff'
# /clock
# /diagnostics
# /gazebo/link_states
# /gazebo/model_states
# /gazebo/parameter_descriptions
# /gazebo/parameter_updates
# /gazebo/set_link_state
# /gazebo/set_model_state


SRV_ARM = '/mavros/cmd/arming'
SRV_SETMODE = '/mavros/set_mode'
SRV_TAKEOFF = '/mavros/cmd/takeoff'
TOPIC_SETPOINT_RAW = '/mavros/setpoint_raw/local'
TOPIC_SETPOINT = '/mavros/setpoint_position/local'
TOPIC_STATE = '/mavros/state'
TOPIC_LOCALPOSITION = '/mavros/local_position/pose'
TOPIC_SETPOINT = '/mavros/setpoint_position/local'
TOPIC_SETVELOCITY = '/mavros/setpoint_velocity/cmd_vel'
TOPIC_MODELSTATE = '/gazebo/model_states'
TOPIC_SET_MODELSTATE =  '/gazebo/set_model_state'
TOPIC_EKF =  '/ekf'
TOPIC_EKF2 =  '/ekf2'
TOPIC_EKF3 =  '/ekf3'

TOPIC_IMGCOMPRESSED = "/camera_red_iris/image_raw/compressed"

FREQ = 20.0
TS = 1.0/FREQ


IS_MOVED = False
INDEX_MOVED = 0
MUL = 4

####################################
############## Params ##############
def setParams():
    rospy.set_param('/idx_Permit_EKF',2*MUL) # Scale ratio (1/anchor_sep)
    rospy.set_param('/S_covariance',0.002*MUL) # Covariance
    rospy.set_param('/R_covariance',0.002*MUL) # Process covariance
    rospy.set_param('/Q_covariance',0.005*MUL) # Measurement covariance



# CONTROLLER CLASS
class FormationControl:


	def __init__(self,names,rate,initial):
		''' Variables '''

		self.psi_1 = 0;
		self.psi_2 = 0;
		self.psi_3 = 0;
		self.index_1 = 0
		self.index_2 = 1
		self.index_3 = 2

		self.dronesName = names
		self.agents = []
		self.detectionList = []
		self.rate = rate
		self.initial_pos = initial
		rospy.rate = rate
		self.fov = 1.3962634  # Field of view of the camera
		# Coefficient  K for the update
		self.k = 0.5
		# Filter Params
		self.idx_seq = 0
		self.r1 = 0.
		self.r2 = 0.
		self.r3 = 0.

		self.s1 = 0.
		self.s2 = 0.
		self.s3 = 0.

		self.x_1 = 0
		self.x_2 = 0
		self.x_3 = 0


		self.y_1 = 0
		self.y_2 = 0
		self.y_3 = 0


		self.rate1 = 0
		self.rate2 = 0
		self.rate3 = 0

		self.gamma1 = 0
		self.gamma2 = 0
		self.gamma3 = 0		

		self.sum1 = 0
		self.sum2 = 0
		self.sum3 = 0


		self.cov = np.zeros((len(names),2,2)) # Covariance
		for i in range(len(names)):
			self.cov[i,:,:] = 0.002*np.identity(2) #rospy.get_param('/S_covariance')*np.identity(2)
		self.R_cov = 0.002*np.identity(2) #rospy.get_param('/R_covariance')*np.identity(2) # Process covariance
		self.Q_cov = 0.005*np.identity(2) #rospy.get_param('/Q_covariance')*np.identity(2) # Measurement covariance
		self.ctr_in = np.zeros((3,len(names))) # Control inputs of all agents
		self.mup = np.zeros((2,len(names))) # Predicted mean
		self.x_Ekf = np.array([[0.,0.,0.] , [0.,0.,0.]]) # EKF result (mu)
		self.G = np.identity(2) # Auxiliary linearized model matrix
		self.H = np.identity(2) # Linearized measurement matrix
		self.y = np.array([[0.,0.,0.] , [0.,0.,0.]]) # Measurements
		self.x_Ekf_Pub = Twist()

		self.x_Ekf_Pub2 = Twist()
		self.x_Ekf_Pub3 = Twist()
		# initialization
		self.setup()
		# configuration
		self.Onarming()
		# Gazebo model State
		self.model_state = rospy.Subscriber( TOPIC_MODELSTATE,ModelStates,self.cb_model_state)
		y = 0
		while not rospy.is_shutdown() and y < 50:
	        	self.fly()
	        	y = y+1
	        	rate.sleep()
		self.pub_ekf = rospy.Publisher(TOPIC_EKF, Twist) # error distance

		self.pub_ekf2 = rospy.Publisher(TOPIC_EKF2, Twist) # R2
		self.pub_ekf3 = rospy.Publisher(TOPIC_EKF3, Twist) # R3

		# # configuration boarding
		# self.Onboarding()
		# # configuration takeOff
		# self.fly()



	######### Gazebo Model State ###############
	########################################################
	def find_index(self,names,iris):
		index = 0
		for x in range(len(names)):
			if names[x] == iris :
				index = x
		return index
			
	# callBack gazebo Model states

	def cb_model_state(self,state):
		self.model_state = state

		index_1 = self.find_index(state.name,'iris_0')
		index_2 = self.find_index(state.name,'iris_1')
		index_3 = self.find_index(state.name,'iris_2')

		self.index_1 = index_1
		self.index_2 = index_2
		self.index_3 = index_3

		self.dr1 = self.model_state.pose[index_1].position    
		self.dr2 = self.model_state.pose[index_2].position  
		self.dr3 = self.model_state.pose[index_3].position 
		# Transformation from  euler_from_quaternion
		q1 = (self.model_state.pose[index_1].orientation.x,self.model_state.pose[index_1].orientation.y,self.model_state.pose[index_1].orientation.z,self.model_state.pose[index_1].orientation.w)  
		q2 = (self.model_state.pose[index_2].orientation.x,self.model_state.pose[index_2].orientation.y,self.model_state.pose[index_2].orientation.z,self.model_state.pose[index_2].orientation.w)  
		q3 = (self.model_state.pose[index_3].orientation.x,self.model_state.pose[index_3].orientation.y,self.model_state.pose[index_3].orientation.z,self.model_state.pose[index_3].orientation.w)  

		## yaw from gazebo model state
		self.yaw1 = tf.transformations.euler_from_quaternion(q1)[2] - pi/2.
		self.yaw2 = tf.transformations.euler_from_quaternion(q2)[2] - pi/2.
		self.yaw3 = tf.transformations.euler_from_quaternion(q3)[2] - pi/2.



	######### initialization of the engines ###############
	########################################################

	def setup(self):

		for x in range(len(self.dronesName)):
			drone  = Drone(x,self.dronesName[x],self.rate,self.initial_pos[x])
			self.agents.append(drone)

	def Onarming(self):
		for drone in self.agents:
			drone.setArm()

	def Onboarding(self):
		for drone in self.agents:
			drone.setMode()

	# Fly the location to the initial point
	def fly(self):
		for drone in self.agents:
			drone.fly()


	# Update the location to the initial point
	def hover(self):
		for x in range(len(self.agents)):
			pos_init = self.initial_pos[x]
			drone  = self.agents[x]
			drone.setPoint(0.,0.)

	########################################################


	# THIS THE MAIN FUNCTION COLEADER
	# update according to the local frame	
	def formationController(self,dv_1,dv_2,dv_3):
		# self.agents[0].model_state   --- gazebo model state
		############################################
		###### dv_1,dv_2,dv_3 are now array of Gamma


		dr1 = self.dr1 
		dr2 = self.dr2  
		dr3 = self.dr3
		# Measurements
		scale_ratio = (self.fov/2)/(800/2.0)
		#print(scale_ratio)
		self.beta_1 = dv_1[0]*scale_ratio #detct1.beta #
		self.beta_2 = dv_2[0]*scale_ratio # detct2.beta
		self.beta_3 = dv_3[0]*scale_ratio # detct3.beta #
		print('Beta ',dv_1,dv_2,dv_3)
		print('Beta ',dv_1[0]*scale_ratio,dv_2[0]*scale_ratio,dv_3[0]*scale_ratio)



		noise = np.random.normal(0,0.1,1)  # noise
		d1 = d2 = d3 = 4.    # desired distance between agents
		self.k_psi = 0.3 # 0.2;
		eta = np.random.normal(0,0.1,3)

		# Distances      
		r1 =  hypot(dr1.x - dr2.x ,dr1.y - dr2.y ) +  eta[0] # Current distance Agent1 - Agent2
		r2 =  hypot(dr2.x - dr3.x ,dr2.y - dr3.y ) +  eta[1]  # Current distance Agent2 - Agent3
		r3 =  hypot(dr3.x - dr1.x ,dr3.y - dr1.y ) +  eta[2]  # Current distance Agent3 - Agent1
		self.r1 = r1

		# Filter params
		self.y[0,0], self.y[1,0] = r1, self.beta_1
		self.y[0,1], self.y[1,1] = r2, self.beta_2
		self.y[0,2], self.y[1,2] = r3, self.beta_3

		# Apply Filter
		for i in range(len(self.dronesName)):
			self.filter_ekf(i)
		# Angles
		# the delta angle inside the triangle 
		self.alpha_1 =  np.arccos((r3*r3 + r1*r1 - r2*r2)/ (2*r3*r1))
		self.alpha_2 =  np.arccos((r1*r1 + r2*r2 - r3*r3)/ (2*r1*r2))
		self.alpha_3 =  np.arccos((r2*r2 + r3*r3 - r1*r1)/ (2*r2*r3))

		# Controller
		s1 =  -(d1 - self.x_Ekf[0,0]) # e1 error distance
		s2 =  -(d2 - self.x_Ekf[0,1]) # e2 error distance
		s3 =  -(d3 - self.x_Ekf[0,2]) # e3 error distance

		# Agents from the Agent_node 
		# Instance of Agent_node for R1,R2,R3
		R1 = self.agents[0].position.pose.position # R1 
		R2 = self.agents[1].position.pose.position # R2
		R3 = self.agents[2].position.pose.position # R3

		# Control signals
		#u_1 = np.array([ s1 * sin(self.beta_1),s1 * cos(self.beta_1)])
		#u_2 = np.array([ s2 * sin(self.beta_2),s2 * cos(self.beta_2)])
		#u_3 = np.array([ s3 * sin(self.beta_3),s3 * cos(self.beta_3)])

		u_1 = np.array([ s1 * sin(self.x_Ekf[1,0]),s1 * cos(self.x_Ekf[1,0])])
		u_2 = np.array([ s2 * sin(self.x_Ekf[1,1]),s2 * cos(self.x_Ekf[1,1])])
		u_3 = np.array([ s3 * sin(self.x_Ekf[1,2]),s3 * cos(self.x_Ekf[1,2])])

		# For Local pose
		x_1 = u_1[0]*TS #*self.k_psi
		y_1 = u_1[1]*TS #*self.k_psi

		x_2 = u_2[0]*TS #*self.k_psi
		y_2 = u_2[1]*TS #*self.k_psi
		        
		x_3 = u_3[0]*TS #*self.k_psi
		y_3 = u_3[1]*TS #*self.k_psi

		# Next yaw rates 
		rate1 = -self.k_psi*self.beta_1;  
		rate2 = -self.k_psi*self.beta_2;
		rate3 = -self.k_psi*self.beta_3;

		# gamma
		self.gamma1 = dv_1[0]
		self.gamma2 = dv_1[1]
		self.gamma3 = dv_1[2]

		# Control inputs to be used in filter
		self.ctr_in[0][0], self.ctr_in[1][0], self.ctr_in[2][0] = x_1, y_1, rate1
		self.ctr_in[0][1], self.ctr_in[1][1], self.ctr_in[2][1] = x_2, y_2, rate2
		self.ctr_in[0][2], self.ctr_in[1][2], self.ctr_in[2][2] = x_3, y_3, rate3

		# Next location publications
		self.agents[0].setPoint(x_1 ,y_1,3.,rate1)
		self.agents[1].setPoint(x_2 ,y_2,3.,rate2)
		self.agents[2].setPoint(x_3 ,y_3,3.,rate3)

		# # publish psi and beta --bag
		self.agents[0].set_psi_beta(self.psi_1,self.beta_1,r1,s1)
		self.agents[1].set_psi_beta(self.psi_2,self.beta_2,r2,s2)
		self.agents[2].set_psi_beta(self.psi_3,self.beta_3,r3,s3)


	def computeX(self,i,w):
		# Compute X
		# Formulate X = i*px(f - w) /f
		px  = -7.656598* 10**-4
		f = 0.3810
		X = i * px * (f - w) / f
		beta =  atan2(X,w)
		return X,beta

	iteration = 0


	# Formation Controller with Real f and px
	def formationController2(self,dv_1,dv_2,dv_3):
		# self.agents[0].model_state   --- gazebo model state
		############################################
		###### dv_1,dv_2,dv_3 are now array of Gamma


		dr1 = self.dr1 
		dr2 = self.dr2  
		dr3 = self.dr3
		noise = np.random.normal(0,0.1,1)  # noise
		d1 = d2 = d3 = 5.    # desired distance between agents
		self.k_psi = 0.5 # 0.3;
		eta = np.random.normal(0,0.1,3)

		# Distances      
		r1 =  hypot(dr1.x - dr2.x ,dr1.y - dr2.y ) +  eta[0] # Current distance Agent1 - Agent2
		r2 =  hypot(dr2.x - dr3.x ,dr2.y - dr3.y ) +  eta[1]  # Current distance Agent2 - Agent3
		r3 =  hypot(dr3.x - dr1.x ,dr3.y - dr1.y ) +  eta[2]  # Current distance Agent3 - Agent1

		self.r1 = r1
		self.r2 = r2
		self.r3 = r3

		# Measurements
		scale_ratio = (self.fov/2)/(800/2.0)
		#print(scale_ratio)
		# beta and the distance from the center of the camera
		X1,self.beta_1 = self.computeX(dv_1[0] ,r1) #dv_1[0]*scale_ratio #detct1.beta #
		X2,self.beta_2 = self.computeX(dv_2[0] ,r2) #dv_2[0]*scale_ratio # detct2.beta
		X2,self.beta_3 = self.computeX(dv_3[0] ,r3) #dv_3[0]*scale_ratio # detct3.beta #
		
		# print('Beta ',self.beta_1,self.beta_2,self.beta_3)
		# print('Beta ',dv_1[0]*scale_ratio,dv_2[0]*scale_ratio,dv_3[0]*scale_ratio)
		# print('Beta ',dv_1[0],dv_2[0],dv_3[0])




		# Filter params
		self.y[0,0], self.y[1,0] = r1, self.beta_1
		self.y[0,1], self.y[1,1] = r2, self.beta_2
		self.y[0,2], self.y[1,2] = r3, self.beta_3

		# Apply Filter
		for i in range(len(self.dronesName)):
			self.filter_ekf(i)
		# Angles
		# the delta angle inside the triangle 
		self.alpha_1 =  np.arccos((r3*r3 + r1*r1 - r2*r2)/ (2*r3*r1))
		self.alpha_2 =  np.arccos((r1*r1 + r2*r2 - r3*r3)/ (2*r1*r2))
		self.alpha_3 =  np.arccos((r2*r2 + r3*r3 - r1*r1)/ (2*r2*r3))

		# Controller
		self.s1 =  -(d1 - self.x_Ekf[0,0]) # e1 error distance
		self.s2 =  -(d2 - self.x_Ekf[0,1]) # e2 error distance
		self.s3 =  -(d3 - self.x_Ekf[0,2]) # e3 error distance
		self.sum1 += self.s1
		self.sum2 += self.s2
		self.sum3 += self.s3

		# Agents from the Agent_node 
		# Instance of Agent_node for R1,R2,R3
		R1 = self.agents[0].position.pose.position # R1 
		R2 = self.agents[1].position.pose.position # R2
		R3 = self.agents[2].position.pose.position # R3

		# gamma
		self.gamma1 = dv_1[0]
		self.gamma2 = dv_2[0]
		self.gamma3 = dv_3[0]		

		# Control signals
		#u_1 = np.array([ s1 * sin(self.beta_1),s1 * cos(self.beta_1)])
		#u_2 = np.array([ s2 * sin(self.beta_2),s2 * cos(self.beta_2)])
		#u_3 = np.array([ s3 * sin(self.beta_3),s3 * cos(self.beta_3)])

		u_1 = np.array([ self.s1 * sin(self.x_Ekf[1,0]),self.s1 * cos(self.x_Ekf[1,0])])
		u_2 = np.array([ self.s2 * sin(self.x_Ekf[1,1]),self.s2 * cos(self.x_Ekf[1,1])])
		u_3 = np.array([ self.s3 * sin(self.x_Ekf[1,2]),self.s3 * cos(self.x_Ekf[1,2])])

		self.iteration += 1

		summ = (self.sum1**2 + self.sum2**2 + self.sum3**2)/3

		#print('ss',self.iteration ,sqrt(summ)/self.iteration)

		# For Local pose
		self.x_1 = u_1[0]*TS #*self.k_psi
		self.y_1 = u_1[1]*TS #*self.k_psi

		self.x_2 = u_2[0]*TS #*self.k_psi
		self.y_2 = u_2[1]*TS #*self.k_psi
		        
		self.x_3 = u_3[0]*TS #*self.k_psi
		self.y_3 = u_3[1]*TS #*self.k_psi


		# Next yaw rates 
		self.rate1 = -self.k_psi*self.beta_1;  
		self.rate2 = -self.k_psi*self.beta_2;
		self.rate3 = -self.k_psi*self.beta_3;

		# Control inputs to be used in filter
		self.ctr_in[0][0], self.ctr_in[1][0], self.ctr_in[2][0] = self.x_1, self.y_1, self.rate1
		self.ctr_in[0][1], self.ctr_in[1][1], self.ctr_in[2][1] = self.x_2, self.y_2, self.rate2
		self.ctr_in[0][2], self.ctr_in[1][2], self.ctr_in[2][2] = self.x_3, self.y_3, self.rate3

		# Next location publications
		self.agents[0].setPoint(self.x_1 ,self.y_1,3.,self.rate1)
		self.agents[1].setPoint(self.x_2 ,self.y_2,3.,self.rate2)
		self.agents[2].setPoint(self.x_3 ,self.y_3,3.,self.rate3)





	def pub_rosvars(self):
		# # publish psi and beta --bag

		self.agents[0].set_psi_beta(self.psi_1,self.beta_1,self.r1,self.s1,self.gamma1)
		self.agents[1].set_psi_beta(self.psi_2,self.beta_2,self.r2,self.s2,self.gamma2)
		self.agents[2].set_psi_beta(self.psi_3,self.beta_3,self.r3,self.s3,self.gamma2)



	def pub_velocities(self):
		self.agents[0].set_velocities(self.x_1,self.y_1,self.rate1)
		self.agents[1].set_velocities(self.x_2,self.y_2,self.rate2)
		self.agents[2].set_velocities(self.x_3,self.y_3,self.rate3)



	######### Filtering #########################
	#############################################
	def filter_ekf(self,i):
		# i: Agent index
		S_cov = self.cov[i,:,:]
		# Wait for some time before running EKF
		if self.idx_seq < 2: #rospy.get_param('/idx_Permit_EKF'):
			self.x_Ekf[:,i] = self.y[:,i]
			return
		# Propagate mu through the nonlinear motion model
		self.motion_model_state(i)
		# Prediction update
		self.linearized_motion_model(i)
		Sp = np.matmul(self.G,np.matmul(S_cov,np.transpose(self.G))) + self.R_cov

		# Measurement update (H is identity)
		LL = np.matmul(self.H , np.matmul(Sp , np.transpose(self.H))) + self.Q_cov
		K = np.matmul(Sp , np.matmul(np.transpose(self.H) , np.linalg.inv(LL)))
		# Predicted measurement
		self.y_mdl = self.mup[:,i]
		# Difference between current and predicted measurements
		I = np.array([[self.y[0,i] - self.mup[0,i]] , [self.y[1,i] - self.mup[1,i]]])
		# print 'ekf: ', self.x_Ekf[:,i]
		# Update belief
		ekf_int = np.array([[self.mup[0,i]],[self.mup[1,i]]]) + np.matmul(K , I)
		self.x_Ekf[0][i], self.x_Ekf[1][i] = ekf_int[0], ekf_int[1]
		# Wrap phi between [-pi,pi)
		self.x_Ekf[1][i] = self.mod_angle(self.x_Ekf[1][i])
		#print self.R1_theta, self.R2_theta, self.x_Ekf[2][0], self.x_Ekf[3][0]

		# Update covariance
		self.cov[i,:,:] = np.matmul((np.identity(2) - np.matmul(K , self.H)) , Sp)

		# Publish the estimated state
		self.x_Ekf_Pub.linear.x = self.r1
		self.x_Ekf_Pub.linear.y = self.x_Ekf[0][0]
		self.x_Ekf_Pub.angular.x = self.beta_1
		self.x_Ekf_Pub.angular.y = self.x_Ekf[1][0]

		# Publish the estimated state
		# For other Agent
		self.x_Ekf_Pub2.linear.x = self.r2
		self.x_Ekf_Pub2.linear.y = self.x_Ekf[0][1]
		self.x_Ekf_Pub2.angular.x = self.beta_2
		self.x_Ekf_Pub2.angular.y = self.x_Ekf[1][1]

		self.x_Ekf_Pub3.linear.x = self.r3
		self.x_Ekf_Pub3.linear.y = self.x_Ekf[0][2]
		self.x_Ekf_Pub3.angular.x = self.beta_3
		self.x_Ekf_Pub3.angular.y = self.x_Ekf[1][2]		


		#self.header_main.seq = self.idx_seq
		#self.header_main.stamp = rospy.Time.now() # Current time as rosTime
		#print self.x_Ekf



	### State model ###
	### X = [d_i,phi_i], U = [vx_i, vy_i, omega_i]
	# d_i,phi_i = x_Ekf[:,i]
	# vx, vy, omega = ctr_in[0:2,i]
	def motion_model_state(self,i):
		# Parameters
		d, phi = self.x_Ekf[0][i], self.x_Ekf[1][i]
		vx, vy, omega = self.ctr_in[0][i], self.ctr_in[1][i], self.ctr_in[2][i]
		# Motion model of the relative state
		self.mup[0][i] = d + (-vx*math.sin(phi) - vy*math.cos(phi))*TS
		self.mup[1][i] = phi + omega*TS + (1./d)*(-vx*math.cos(phi) + vy*math.sin(phi))*TS
		return 0



	### Linearized motion model ###
	### X = [d_i,phi_i], U = [vx_i, vy_i, omega_i]
	# d_i,phi_i = x_Ekf[:,i]
	# vx, vy, omega = ctr_in[0:2,i]
	def linearized_motion_model(self,i):
		# Parameters
		d, phi = self.mup[0][i], self.mup[1][i]
		vx, vy, omega = self.ctr_in[0][i], self.ctr_in[1][i], self.ctr_in[2][i]
		# Motion model of the relative state
		self.G[0][0] = 1.
		self.G[0][1] = (-vx*math.cos(phi) + vy*math.sin(phi))*TS
		self.G[1][0] = -(1./pow(d,2.))*(-vx*math.cos(phi) + vy*math.sin(phi))*TS
		self.G[1][1] = 1. + (1./d)*(vx*math.sin(phi) + vy*math.cos(phi))*TS
		return 0


	### Mod of angle ###
	def mod_angle(self,x):
		# Params
		y = x
		if x > np.pi:
			y = x % np.pi
		else:
			if x <= -np.pi:
				y = (x + 2*np.pi) % np.pi
		return y





# Main function
def main():

    # Initializing node
    rospy.init_node('main_node', anonymous=True)

    ##### list of uavs
    ##### Number of agents are set here
    Names = ['uav0','uav1','uav2'] 
    # ROS loop rate, [Hz]
    rate = rospy.Rate(FREQ)
    initial_pos = [[0.0,0.0,3.,0.],[5.0,0.0,3.,2.3],[2.0,4.0,3.,-2.5]]
    # Controller 
    cnt = FormationControl(Names,rate,initial_pos)
    ## Taking off
    cnt.Onboarding()
    cnt.fly()

    ##Altitude stabilization
    #################################
    k = 0
    is_z_ok = False
    while not (is_z_ok):
    	is_ok = True
    	for dr in cnt.agents:
    		diff_z = dr.position.pose.position.z - 3.0
    		if(abs(diff_z) > 0.1 ):
    			is_ok = False
    	    	
    	cnt.hover()
    	rate.sleep()
    	is_z_ok = is_ok

    i = 0
    ################################

    gz_state_it  = 0 # For gazebo states every 2


    while not rospy.is_shutdown():
    	now = rospy.get_rostime()
    	#rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    	# cnt.updateLocation()
    	dv_1 = cnt.agents[0].process_img() # Pixel detection Gamma
    	dv_2 = cnt.agents[1].process_img() # Pixel detection Gamma
    	dv_3 = cnt.agents[2].process_img() # Pixel detection Gamma
    	cnt.formationController2(dv_1, dv_2, dv_3) # Controller algorithm
    	cnt.idx_seq = cnt.idx_seq + 1 # Iteration index
    	# Publish results ekfs
    	cnt.pub_ekf.publish(cnt.x_Ekf_Pub)
    	cnt.pub_ekf2.publish(cnt.x_Ekf_Pub2)
    	cnt.pub_ekf3.publish(cnt.x_Ekf_Pub3)
    	cnt.pub_rosvars()
    	cnt.pub_velocities()

        # gazeboModel State as an array
        gz_state_it  = gz_state_it +1 
        cnt.agents[0].pub_gazeboStates(cnt.agents[0].model_state)


        # if gz_state_it % 2 == 0 :
             

    	rate.sleep()



if __name__ == '__main__':
	try:
		print('starting ......')
		main()
	except rospy.ROSInterruptException:
		pass