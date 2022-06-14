
#!/usr/bin/env python

import rospy  
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError 
import cv2  
import numpy as np
from std_msgs.msg import String 



#This class will receive a ROS image and transform it to opencv format  
class TrafficSignalDetection():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image 
        self.vel=Twist() #Robot's speed 

        ############################### SUBSCRIBERS #####################################  
        image_sub = rospy.Subscriber("video_source/raw", Image, self.image_cb) 
        self.pub_traffic_shape = rospy.Publisher('traffic_signal_detection', Image, queue_size=1)
        self.pub_traffic_signal = rospy.Publisher('traffic_signal_recognition', String, queue_size=1)  



        #********** INIT NODE **********###  
        r = rospy.Rate(10) #10Hz  
        while not rospy.is_shutdown():  
            if self.image_received:                 
                #I resized the image so it can be easier to work with 
                cv_image = self.cv_image
                cv_image = cv2.resize(cv_image,(300,300))
                cv_image = cv_image[35:200, 100:300]

                hsv_blue_min = (95,100,100)
                hsv_blue_max = (130,255,255)

                hsv_red_min1 = (0,70,70)  #0,88,179
                hsv_red_max1 = (33,255,255)
                hsv_red_min2 = (155,70,90)
                hsv_red_max2 = (180,255,255)


                hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)


                mask_blue = cv2.inRange(hsv, hsv_blue_min, hsv_blue_max)

                mask_red1 = cv2.inRange(hsv, hsv_red_min1, hsv_red_max1)
                mask_red2 = cv2.inRange(hsv, hsv_red_min2, hsv_red_max2)
                mask_red = mask_red1 + mask_red2

                   
                (contoursblue, hierarchy_b) = cv2.findContours(mask_blue,1, cv2.CHAIN_APPROX_NONE)     # Find contours blue
                (contoursred,  hierarchy_r) = cv2.findContours(mask_red,1, cv2.CHAIN_APPROX_NONE)      # Find contours red


                ##### Check if there is a signal blue and crop this section 

                if len(contoursblue) > 0:
                    c_blue = max(contoursblue, key=cv2.contourArea)
                    areablue = cv2.contourArea(c_blue) 
                else:
                    c_blue = 0
                    areablue = 0

                if len(contoursred) > 0:
                    c_red = max(contoursred, key=cv2.contourArea)
                    areared = cv2.contourArea(c_red)
                else:
                    c_red = 0
                    areared = 0

                    
                if(areablue > 375.0):  #2500
                    x,y,w,h = cv2.boundingRect(c_blue)
                    #cv2.rectangle(cv_image,(x,y),(x+w,y+h), (255,0,0),2)
                    image_detected = cv_image[y:y+h, x:x+w]
                    image_detected = cv2.resize(image_detected,(30,30))
                    image_detected = self.bridge_object.cv2_to_imgmsg(image_detected,'rgb8')
                    self.pub_traffic_shape.publish(image_detected)
                    print("send")


                ##### Check if there is a signal red and crop this section 
                
                elif(areared > 650.0): #1100
                    x,y,w,h = cv2.boundingRect(c_red)
                    #cv2.rectangle(cv_image,(x,y),(x+w,y+h), (255,0,0) ,2)
                    image_detected = cv_image[y+1:y+h-1, x+1:x+w-1]
                    image_detected = cv2.resize(image_detected,(30,30))
                    image_detected = self.bridge_object.cv2_to_imgmsg(image_detected,'rgb8')
                    self.pub_traffic_shape.publish(image_detected)
                    print("send")


                else:
                        self.pub_traffic_signal.publish("Nothing")




            r.sleep()  
        cv2.destroyAllWindows() 

 

    def image_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format 
        try: 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="rgb8")      
            self.image_received = 1 #Turn the flag on 

        except CvBridgeError as e: 
            print(e) 

         
    def cleanup(self):  
        cv2.destroyAllWindows() 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("traffic_signal_detection", anonymous=True)  
    TrafficSignalDetection()  
