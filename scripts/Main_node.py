#!/usr/bin/env python



import rospy

from geometry_msgs.msg import Point, PoseStamped,TwistStamped
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

TOPIC_IMGCOMPRESSED = "/camera_red_iris/image_raw/compressed"

FREQ = 20.0
TS = 1.0/FREQ


IS_MOVED = False
INDEX_MOVED = 0





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
		self.fov = 0
		# Coefficient  K for the update
		self.k = 0.5

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

		q1 = (self.model_state.pose[index_1].orientation.x,self.model_state.pose[index_1].orientation.y,self.model_state.pose[index_1].orientation.z,self.model_state.pose[index_1].orientation.w)  
		q2 = (self.model_state.pose[index_2].orientation.x,self.model_state.pose[index_2].orientation.y,self.model_state.pose[index_2].orientation.z,self.model_state.pose[index_2].orientation.w)  
		q3 = (self.model_state.pose[index_3].orientation.x,self.model_state.pose[index_3].orientation.y,self.model_state.pose[index_3].orientation.z,self.model_state.pose[index_3].orientation.w)  

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

		dr1 = self.dr1 
		dr2 = self.dr2  
		dr3 = self.dr3 

		noise = np.random.normal(0,0.1,1)  # noise
		d1 = d2 = d3 = 6.    # desired distance between agents
		self.k_psi = 1 # 0.2;
		eta = np.random.normal(0,0.1,3)
	    
	    # Distances      
		r1 =  hypot(dr1.x - dr2.x ,dr1.y - dr2.y ) #+  ta[0] # Current distance Agent1 - Agent2
		r2 =  hypot(dr2.x - dr3.x ,dr2.y - dr3.y ) #+  eta[1]  # Current distance Agent2 - Agent3
		r3 =  hypot(dr3.x - dr1.x ,dr3.y - dr1.y ) #+  eta[2]  # Current distance Agent3 - Agent1
	    
	     # Angles
	    # the delta angle inside the triangle
		self.alpha_1 =  np.arccos((r3*r3 + r1*r1 - r2*r2)/ (2*r3*r1))
		self.alpha_2 =  np.arccos((r1*r1 + r2*r2 - r3*r3)/ (2*r1*r2))
		self.alpha_3 =  np.arccos((r2*r2 + r3*r3 - r1*r1)/ (2*r2*r3))
	    
	   #  Controller
		s1 =  -(d1 - r1) #  e1 error distance
		s2 =  -(d2 - r2)  # e2 error distance
		s3 =  -(d3 - r3) # e3 error distance


		R1 = self.agents[0].position.pose.position 
		R2 = self.agents[1].position.pose.position
		R3 =  self.agents[2].position.pose.position  

		scale_ratio = (self.fov/2)/(800/2.0)
		self.beta_1 = dv_1*scale_ratio #detct1.beta #
		self.beta_2 = dv_2*scale_ratio # detct2.beta
		self.beta_3 = dv_3*scale_ratio # detct3.beta #
		print('Beta ',self.beta_1,self.beta_2,self.beta_3)

		u_1 = np.array([ s1 * sin(self.beta_1),s1 * cos(self.beta_1)])
		u_2 = np.array([ s2 * sin(self.beta_2),s2 * cos(self.beta_2)])
		u_3 = np.array([ s3 * sin(self.beta_3),s3 * cos(self.beta_3)])

		# For Local pose
		x_1 = u_1[0] *TS #*self.k_psi
		y_1 = u_1[1]*TS #*self.k_psi
	    
		x_2 = u_2[0]*TS #*self.k_psi
		y_2 = u_2[1]*TS #*self.k_psi
	            
		x_3 = u_3[0]*TS #*self.k_psi
		y_3 = u_3[1]*TS #*self.k_psi


		rate1 = -self.k_psi*self.beta_1;
		rate2 = -self.k_psi*self.beta_2;
		rate3 = -self.k_psi*self.beta_3;
		# Next location publications
		self.agents[0].setPoint(x_1 ,y_1,R1.z,rate1)
		self.agents[1].setPoint(x_2 ,y_2,R2.z,rate2)
		self.agents[2].setPoint(x_3 ,y_3,R3.z,rate3)

		# # publish psi and beta --bag
		self.agents[0].set_psi_beta(self.psi_1,self.beta_1,r1,s1)
		self.agents[1].set_psi_beta(self.psi_2,self.beta_2,r2,s2)
		self.agents[2].set_psi_beta(self.psi_3,self.beta_3,r3,s3)



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

    cnt.Onboarding()
    cnt.fly()

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



    while not rospy.is_shutdown():
    	now = rospy.get_rostime()
    	rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    	# cnt.updateLocation()
    	dv_1 = cnt.agents[0].process_img()
    	dv_2 = cnt.agents[1].process_img()
    	dv_3 = cnt.agents[2].process_img()
    	cnt.formationController(dv_1, dv_2, dv_3)
    	rate.sleep()



if __name__ == '__main__':
	try:
		print('starting ......')
		main()
	except rospy.ROSInterruptException:
		pass