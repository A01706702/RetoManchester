#!/usr/bin/env python

import rospy  
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError 
import cv2  
import numpy as np
from std_msgs.msg import String 
#from tensorflow import keras
#from tensorflow.keras import layers
#import tensorflow as tf



#This class will receive a ROS image and transform it to opencv format  
class TrafficSignal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image 
        self.vel=Twist() #Robot's speed 

        #model = keras.models.load_model("signal_traffic_classifier.h5")

        ############################### SUBSCRIBERS #####################################  
        image_sub = rospy.Subscriber("video_source/raw", Image, self.image_cb) 
        self.pub_traffic_light = rospy.Publisher('traffic_signal', String, queue_size=1)  


        #********** INIT NODE **********###  
        r = rospy.Rate(10) #10Hz  
        while not rospy.is_shutdown():  
            if self.image_received: 

                #I resized the image so it can be easier to work with 
                cv_image = self.cv_image
                cv_image = cv2.resize(cv_image,(300,300))
                cv_image = cv_image[50:300, 100:300]

                hsv_blue_min = (95,50,20)
                hsv_blue_max = (130,255,255)


                hsv_red_min1 = (0,50,50)
                hsv_red_max1 = (10,255,255)
                hsv_red_min2 = (155,25,50)
                hsv_red_max2 = (180,255,255)



                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


                mask_blue = cv2.inRange(hsv, hsv_blue_min, hsv_blue_max)
                mask_red1 = cv2.inRange(hsv, hsv_red_min2, hsv_red_max2)
                mask_red2 = cv2.inRange(hsv, hsv_red_min1, hsv_red_max1)

                mask_red = mask_red1 + mask_red2
                


                (contoursblue, hierarchy_b) = cv2.findContours(mask_blue,1, cv2.CHAIN_APPROX_NONE)     # Find contours blue
                (contoursred,  hierarchy_r) = cv2.findContours(mask_red,1, cv2.CHAIN_APPROX_NONE)      # Find contours red


                ##### Check if there is a signal blue and crop this section to predict with the cnn model #####
                if len(contoursblue) > 0:
                    c = max(contoursblue, key=cv2.contourArea)
                    areablue = cv2.contourArea(c)
                    
                    
                    if(areablue > 2500.0):
                        x,y,w,h = cv2.boundingRect(c)
                        cv2.rectangle(cv_image,(x,y),(x+w,y+h), (255,0,0) ,2)
                        image_detected = cv_image[y:y+h, x:x+w]
                        image_detected = cv2.resize(image_detected,(30,30))

                        #### Here we have to call the predictor of the CNN sending image_detected as input ####
                        



                        print("Detected blue")


                ##### Check if there is a signal red and crop this section to predict with the cnn model #####
                if len(contoursred) > 0:
                    c = max(contoursred, key=cv2.contourArea)
                    areared = cv2.contourArea(c)
                    print(areared)
                    
                    if(areared > 1100.0):
                        x,y,w,h = cv2.boundingRect(c)
                        cv2.rectangle(cv_image,(x,y),(x+w,y+h), (255,0,0) ,2)
                        image_detected = cv_image[y:y+h, x:x+w]
                        image_detected = cv2.resize(image_detected,(30,30))

                        #### Here we have to call the predictor of the CNN sending image_detected as input #####
                        self.pub_traffic_light.publish("Stop")

                        print("Detected red")




                #cv2.imshow("Image",cv_image)
                #cv2.waitKey(3)

 




                




            r.sleep()  
        cv2.destroyAllWindows() 

 

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
        cv2.destroyAllWindows() 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("traffic_signal", anonymous=True)  
    TrafficSignal()  