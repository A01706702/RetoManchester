#!/usr/bin/env python  

import rospy  
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError 
import cv2  
import numpy as np
from std_msgs.msg import String 


#This class will receive a ROS image and transform it to opencv format  
class TrafficLight():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image 
        self.vel=Twist() #Robot's speed 


        ############################### SUBSCRIBERS #####################################  
        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb) 
        self.pub_traffic_light = rospy.Publisher('traffic_light', String, queue_size=1)  


        #********** INIT NODE **********###  
        r = rospy.Rate(10) #10Hz  
        while not rospy.is_shutdown():  
            if self.image_received: 

                #I resized the image so it can be easier to work with 
                cv_image = self.cv_image
                cv_image = cv2.resize(cv_image,(300,300))
                cv_image = cv_image[35:200, :]

                #Once we read the image we need to change the color space to HSV 
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 

     
		        #HSV Limits
                min_green = np.array([49,50,150])  #49,39,130
                max_green = np.array([98,255,255])


                min_red1 = np.array([0,88,179])  #0,88,179
                max_red1 = np.array([33,255,255])
                min_red2 = np.array([155,70,90])  
                max_red2 = np.array([180,255,255]) 



     
                #This is the actual color detection  
                #Here we will create a mask that contains only the colors defined in your limits 
                #This mask has only one dimension, so its black and white 
                mask_g = cv2.inRange(hsv, min_green, max_green) 

                mask_r1 = cv2.inRange(hsv, min_red1, max_red1)
                mask_r2 = cv2.inRange(hsv, min_red2, max_red2)
                mask_r = mask_r1 + mask_r2

                #We use the mask with the original image to get the colored post-processed image 
                res_g = cv2.bitwise_and(cv_image,cv_image, mask= mask_g) 
                res_r = cv2.bitwise_and(cv_image,cv_image, mask= mask_r)


                # THRESHOLDING
                ret_green, th_green = cv2.threshold(res_g, 0, 255, cv2.THRESH_BINARY)
                ret_red, th_red = cv2.threshold(res_r, 0, 255, cv2.THRESH_BINARY)

                #MORPHOLOGICAL OPERATIONS
                kernel = np.ones((3,3), np.uint8)

                #erosion_green = cv2.erode(th_green, kernel, iterations=2)
                img_green = cv2.dilate(th_green, kernel, iterations=4)

                img_red = cv2.dilate(th_red, kernel, iterations=2)

                


                #BLOB DETECTOR RED
                paramsred = cv2.SimpleBlobDetector_Params()
                paramsred.filterByArea = True
                paramsred.minArea= 135
                paramsred.maxArea= 450
                paramsred.filterByCircularity = True
                paramsred.minCircularity = 0.4
                paramsred.maxCircularity = 1
                paramsred.blobColor = 255

                #BLOB DETECTOR GREEN
                paramsgreen = cv2.SimpleBlobDetector_Params()
                paramsgreen.filterByArea = True
                paramsgreen.minArea= 200  #150
                paramsgreen.maxArea= 450
                paramsgreen.filterByCircularity = True
                paramsgreen.minCircularity = 0.7
                paramsgreen.maxCircularity = 1
                paramsgreen.blobColor = 255

                detectorred = cv2.SimpleBlobDetector_create(paramsred)
                detectorgreen = cv2.SimpleBlobDetector_create(paramsgreen)

                keypoints_green = detectorgreen.detect(img_green)
                keypoints_red = detectorred.detect(img_red)


                if(keypoints_red):
                    print("found_red")
                    self.pub_traffic_light.publish("Red") #publish the robot's speed 
                if(keypoints_green):
                    print("found green")
                    self.pub_traffic_light.publish("Green") #publish the robot's speed 
                
                if((not keypoints_green) and (not keypoints_red)):
                    print("Nothing")
                    self.pub_traffic_light.publish("None") #publish the robot's speed


                #cv2.imshow("Mask Red",img_red)
                #cv2.waitKey(1) 
                #cv2.imshow("Image", cv_image)
                #cv2.waitKey(1)
                #cv2.imshow("Mask Green", img_green)
                #cv2.waitKey(1)


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
    rospy.init_node("traffic_light", anonymous=True)  
    TrafficLight()  
