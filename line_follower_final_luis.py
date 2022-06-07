#!/usr/bin/env python  

import rospy  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2  
from geometry_msgs.msg import Twist
from numpy.polynomial import Polynomial as P
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32 


#This class will make the puzzlebot move following a square 
class LineFollowerClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  

        ############ CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 
        self.theta=0 #angle 
        self.wr=0 
        self.wl=0 
        self.vel = Twist()

        ########## CONSTANTES DEL CONTROLADOR #############
        kp = 0.001 # 0.007
        kd = 0.001  # 0.002
        error = 0
        error_prev = 0


        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image 



        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
 
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        image_sub = rospy.Subscriber("video_source/raw", Image, self.image_cb) 
 

        #********** INIT NODE **********###  
        freq=10.0
        rate = rospy.Rate(freq) #20Hz  
        dt =1/freq #Dt is the time between one calculation and the next one 
        print("Node initialized 20hz") 


        while not rospy.is_shutdown():
            
            v=r*(self.wr+self.wl)/2 
            w=r*(self.wr-self.wl)/L 
            self.theta = self.theta + (w*dt)
            self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))                
            
            if self.image_received: 

               #I resized the image so it can be easier to work with 
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
                        print(error)

                        if (error < 50 and error > -50):
                            self.vel.linear.x = 0.15  ##0.2
                            self.vel.angular.z = 0
                        else:           
                            self.vel.linear.x = 0.05 ## 0.1
                            u= (kp* -float(error)) + (kd* -float(((error-error_prev)/dt))) # Variable de control
                            if(u>0.5):
                                u=0.3
                            elif(u<-0.5):
                                u=-0.3
                            self.vel.angular.z = u

                        error_prev = error
                    else:
                        self.vel.linear.x = 0.15  ##0.2
                        self.vel.angular.z = 0
                        print("No line")
                        
                        



                    self.pub_cmd_vel.publish(self.vel)

          
            rate.sleep()  

    def wl_cb(self, wl):  
        ## This function receives a number   
        self.wl = wl.data 

    def image_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format   
        try: 
            #print("received ROS image, I will convert it to opencv") 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
            self.image_received = 1 #Turn the flag on 
        except CvBridgeError as e: 
            print(e) 

        
    def wr_cb(self, wr):  
        ## This function receives a number.  
        self.wr = wr.data  

    def cleanup(self):  
        vel=Twist()
        self.wl = 0
        self.wr = 0
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        self.pub_cmd_vel.publish(vel)
        print ("Movement Stopped") 




        












############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("line_follower", anonymous=True)  
    LineFollowerClass()
