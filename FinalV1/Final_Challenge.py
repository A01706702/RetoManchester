#!/usr/bin/env python  

import rospy  

from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 
import math
from std_msgs.msg import String 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import cv2  
import time


class Robot():

	#This class implements the differential drive model of the robot 
	def __init__(self): 
		############ ROBOT CONSTANTS ################  
		self.r=0.05 #wheel radius [m] 
		self.L = 0.19 #wheel separation [m] 
		############ Variables ############### 
		self.x = 0.0 #x position of the robot [m] 
		self.y = 0.0 #y position of the robot [m] 
		self.theta = 0.0 #angle of the robot [rad] 

	def update_state(self, wr, wl, delta_t): 
		#This function returns the robot's state 
		#This functions receives the wheel speeds wr and wl in [rad/sec]  
		# and returns the robot's state 
		v=self.r*(wr+wl)/2 
		w=self.r*(wr-wl)/self.L 
		self.theta=self.theta + w*delta_t 

		#Crop theta_r from -pi to pi 
		self.theta=np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
		vx=v*np.cos(self.theta) 
		vy=v*np.sin(self.theta) 
		self.x=self.x+vx*delta_t  
		self.y=self.y+vy*delta_t 

class AutonomousDriving():  
	def __init__(self):  
		rospy.on_shutdown(self.cleanup) 
		self.robot=Robot() #create an object of the Robot class
		self.bridge_object = CvBridge() # create the cv_bridge object 



		### ------------CONSTANTS -------------- ###
		v_msg=Twist() #Robot's desired speed
		self.wr=0 #right wheel speed [rad/s] 
		self.wl=0 #left wheel speed [rad/s] 
		self.current_state = 'line_follower' #Robot's current state
		self.last_signal =""
		self.max_speed = 0.5
		self.normal_speed = 0.2 
		self.traffic_light = "None"
		self.traffic_signal = "None"
		self.error_prev=0
		self.image_received = 0 #Flag to indicate that we have already received an image 
		self.first_Ahead = True
		self.first_Turn = True
		self.vel_linefollower = 0.15 #0.1



		### ----------- PUBLISHERS ------------- ###
		self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 

		############################### SUBSCRIBERS #####################################  
		rospy.Subscriber("wl", Float32, self.wl_cb)  
		rospy.Subscriber("wr", Float32, self.wr_cb)
		rospy.Subscriber("traffic_light", String, self.traffic_light_cb)
		rospy.Subscriber("traffic_signal_recognition", String, self.traffic_signal_cb)
		rospy.Subscriber("video_source/raw", Image, self.image_cb) 


		#********** INIT NODE **********###  
		freq=10 
		rate = rospy.Rate(freq) #freq Hz  
		Dt =1.0/float(freq) #Dt is the time between one calculation and the next one 

		################ MAIN LOOP ################  
		while not rospy.is_shutdown():  

			self.robot.update_state(self.wr, self.wl, Dt) #update the robot's state IMPORTANT!! CALL IT every loop

			############ LINE FOLLOWER BEHAVIOR ############


			### STATE OF LINE FOLLOWER ###
			if(self.current_state == "line_follower"):
				if((self.last_signal == "Stop") or (self.traffic_light == "Red")):
					self.current_state = "Stop"

				elif((self.traffic_light == "Green") and (self.last_signal == "Ahead")):
					self.current_state = "Ahead"

				elif((self.traffic_light == "Green") and (self.last_signal == "Turn right")):
					self.current_state = "Turn right"

				else:
					v,w = self.compute_line_follower()
					v_msg.linear.x = v
					v_msg.angular.z = w
					self.pub_cmd_vel.publish(v_msg)

			### STATE AHEAD ###

			if(self.current_state == "Ahead"):
				if(self.at_goal(0.40, 0.0)):
					self.current_state = "line_follower"

				else:
					v,w = self.compute_ahead()
					v_msg.linear.x = v
					v_msg.angular.z = w
					self.pub_cmd_vel.publish(v_msg)


			### STATE TURN RIGHT ###
			if(self.current_state == "Turn right"):
				rospy.sleep(1)
				v_msg.linear.x = 0.12
				v_msg.angular.z = 0
				self.pub_cmd_vel.publish(v_msg)
				rospy.sleep(3)
				v_msg.linear.x = 0
				v_msg.angular.z = -0.1
				self.pub_cmd_vel.publish(v_msg)
				rospy.sleep(3)
				v_msg.linear.x = 0.1
				v_msg.angular.z = 0
				self.pub_cmd_vel.publish(v_msg)
				rospy.sleep(1)
				v_msg.linear.x = 0
				v_msg.angular.z = 0
				self.pub_cmd_vel.publish(v_msg)
				self.current_state = "line_follower"



			### STATE STOP ###

			if(self.current_state == "Stop"):
				if((self.traffic_light == "Green") and (self.last_signal == "Turn right")):
					self.current_state = "Turn right"
					#print("Turn")
				elif((self.traffic_light == "Green") and (self.last_signal == "Ahead")):
					self.current_state = "Ahead"
					#print("Ahead")

				elif(self.traffic_light == "Green"):
					self.current_state = "line_follower"
					#print("Line follower")

				else:
					v_msg.linear.x = 0
					v_msg.angular.z = 0
					self.pub_cmd_vel.publish(v_msg)


			print(self.current_state)
  

			rate.sleep() 
 

	#### COMPUTE THE LINE FOLLOWER VELOCITIES ####
	def compute_line_follower(self):
			
		###### CONTROLLER CONSTANTS ####
		kp = 0.001  #0.001 
		kd = 0.0001
		dt = 1.0/10.0
		v=0
		w=0

		if self.image_received: 
			image = self.cv_image
			image = cv2.resize(image,(480,480))
			crop_img = image[380:480, 120:360]

			height, width, channels = crop_img.shape
			gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
			blur = cv2.GaussianBlur(gray,(5,5),0)

			ret, thresh1 = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
			mask = cv2.erode(thresh1, None, iterations=2)
			mask = cv2.dilate(mask,None, iterations=2)

			(contours, hierarchy) = cv2.findContours(mask.copy(),1, cv2.CHAIN_APPROX_NONE)  

			if len(contours) > 0:
				c = max(contours, key=cv2.contourArea)
				M = cv2.moments(c)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				
				cv2.line(crop_img, (cx,0),(cx,720),(255,0,0),1)
				cv2.line(crop_img, (0,cy),(1280,cy),(255,0,0),1)
				cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
				

				if cx is not None:
					##### CONTROLADOR PD ######
					error = cx - width/2
					#print(error)

					if (error < 50 and error > -50):
						v = self.vel_linefollower  
						w = 0
					else:           
						v = 0.1#self.vel_linefollower ## 0.1
						u= (kp* -float(error)) + (kd* -float(((error-self.error_prev)/dt))) # Variable de control
						if(u>0.3):
							u=0.3
						elif(u<-0.3):
							u=-0.3
						w = u

					self.error_prev = error
				else:
					v = 0.0  ##0.2
					w = 0.0
					print("No line")

			else:
				v = 0.0
				w = 0.0
					  
		return v,w

	def compute_ahead(self):
		#Restart the position and angle of the robot in order to indicate new origin in the current position
		if(self.first_Ahead):
			self.robot.theta= 0
			self.robot.x= 0 
			self.robot.y= 0
			self.first_Ahead = False

		v = self.vel_linefollower
		w = 0

		return v,w


	def compute_turn_right(self):
		if(self.first_Turn):
			self.robot.theta= 0
			self.robot.x= 0 
			self.robot.y= 0
			self.first_Turn = False


		v = 0.15
		w = -0.08

		return v,w



	def at_goal(self, x_target, y_target):
		if(np.sqrt((x_target - self.robot.x)**2 + (y_target - self.robot.y)**2) < 0.05):
			self.first_Ahead = True
			self.first_Turn = True
			return True
		else:
			return False
		
		


	###################### CALLBACKS #################

	def wl_cb(self, wl):  
		## This function receives a the left wheel speed [rad/s] 
			self.wl = wl.data 

	def wr_cb(self, wr):  
		## This function receives a the right wheel speed.  
		self.wr = wr.data 

	def traffic_light_cb(self, msg):
		self.traffic_light = msg.data

	def traffic_signal_cb(self, msg):
		self.traffic_signal = msg.data
		if(self.traffic_signal != "Nothing"):
			self.last_signal = self.traffic_signal

		if(self.traffic_signal == "No Speed limit"):
			self.vel_linefollower = 0.15


	def image_cb(self, ros_image):  
		## This function receives a ROS image and transforms it into opencv format   
		try: 
			#print("received ROS image, I will convert it to opencv") 
			# We select bgr8 because its the OpenCV encoding by default 
			self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
			self.image_received = 1 #Turn the flag on 
		except CvBridgeError as e: 
			print(e)  

	def cleanup(self):  
		#This function is called just before finishing the node  
		# You can use it to clean things up before leaving  
		# Example: stop the robot before finishing a node.    
		vel_msg = Twist() 
		self.pub_cmd_vel.publish(vel_msg) 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
	rospy.init_node("autonomous_driving", anonymous=True)  
	AutonomousDriving()  
