#!/usr/bin/env python3

import rospy  
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError 
import cv2  
import numpy as np
from std_msgs.msg import String 
from tensorflow import keras
from tensorflow.keras import layers
import tensorflow as tf





#This class will receive a ROS image and transform it to opencv format  
class TrafficSignal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image 
        self.vel=Twist() #Robot's speed 

        model = keras.models.load_model("signal_traffic_classifier_modelv2.h5")

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
                cv_image = cv_image[35:200, 100:300]
                output = cv_image.copy()


                


                hsv_blue_min = (95,100,100)
                hsv_blue_max = (130,255,255)



                hsv_red_min1 = (0,80,170)  #0,88,179
                hsv_red_max1 = (33,255,255)
                hsv_red_min2 = (155,70,90)
                hsv_red_max2 = (180,255,255)


                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


                mask_blue = cv2.inRange(hsv, hsv_blue_min, hsv_blue_max)

                mask_red1 = cv2.inRange(hsv, hsv_red_min1, hsv_red_max1)
                mask_red2 = cv2.inRange(hsv, hsv_red_min2, hsv_red_max2)
                mask_red = mask_red1 + mask_red2

                
                


                (contoursblue, hierarchy_b) = cv2.findContours(mask_blue,1, cv2.CHAIN_APPROX_NONE)     # Find contours blue
                (contoursred,  hierarchy_r) = cv2.findContours(mask_red,1, cv2.CHAIN_APPROX_NONE)      # Find contours red


                ##### Check if there is a signal blue and crop this section to predict with the cnn model #####
                if len(contoursblue) > 0:
                    c = max(contoursblue, key=cv2.contourArea)
                    areablue = cv2.contourArea(c)
                    #print(areablue)

                    
                    if(areablue > 800.0):  #2500
                        x,y,w,h = cv2.boundingRect(c)
                        #cv2.rectangle(cv_image,(x,y),(x+w,y+h), (255,0,0),2)
                        image_detected = cv_image[y:y+h, x:x+w]
                        image_detected = cv2.resize(image_detected,(30,30))
                        image_detected = cv2.cvtColor(image_detected, cv2.COLOR_BGR2RGB)

                        #### Here we have to call the predictor of the CNN sending image_detected as input ####
                        data=[]
                        data.append(np.array(image_detected))
                        X_test = np.array(data)
                        
                        pred = np.argmax(model.predict(X_test), axis=-1)
                        #print(model.predict(X_test))

                        #cv2.imshow("ImageDetected",image_detected)
                        #cv2.waitKey(1)

                        

                        if(pred == 0):
                            print("Stop")
                            self.pub_traffic_light.publish("Stop")
                        if(pred == 1):
                            print("No Speed limit")
                            self.pub_traffic_light.publish("No Speed limit")
                        if(pred == 2):
                            print("Turn right")
                            self.pub_traffic_light.publish("Turn right")
                        if(pred == 3):
                            print("Ahead")
                            self.pub_traffic_light.publish("Ahead")



                        


                ##### Check if there is a signal red and crop this section to predict with the cnn model #####
                if len(contoursred) > 0:
                    c = max(contoursred, key=cv2.contourArea)
                    areared = cv2.contourArea(c)
                    #print(areared)
                    
                    if(areared > 125.0): #1100
                        x,y,w,h = cv2.boundingRect(c)
                        #cv2.rectangle(cv_image,(x,y),(x+w,y+h), (255,0,0) ,2)
                        image_detected = cv_image[y-15:y+h, x:x+w]
                        image_detected = cv2.resize(image_detected,(30,30))
                        image_detected = cv2.cvtColor(image_detected, cv2.COLOR_BGR2RGB)


                        #### Here we have to call the predictor of the CNN sending image_detected as input #####
                        #self.pub_traffic_light.publish("Stop")

                        data=[]
                        data.append(np.array(image_detected))
                        X_test = np.array(data)

                        pred = np.argmax(model.predict(X_test), axis=-1)
                        #print(model.predict(X_test))

                        #cv2.imshow("ImageDetected",image_detected)
                        #cv2.waitKey(1)


                        if(pred == 0):
                            print("Stop")
                            self.pub_traffic_light.publish("Stop")
                        if(pred == 1):
                            print("No Speed limit")
                            self.pub_traffic_light.publish("No Speed limit")
                        if(pred == 2):
                            print("Turn right")
                            self.pub_traffic_light.publish("Turn right")
                        if(pred == 3):
                            print("Ahead")
                            self.pub_traffic_light.publish("Ahead")


                #cv2.imshow("Image",cv_image)
                #cv2.waitKey(1)

                #cv2.imshow("Mask", mask_blue)
                #cv2.waitKey(1)



            r.sleep()  
        cv2.destroyAllWindows() 

 

    def image_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format 
        
        
        try: 
            #print("received ROS image, I will convert it to opencv") 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
            
            #self.cv_image = ros_numpy.numpify(ros_image)
            self.image_received = 1 #Turn the flag on 

        except CvBridgeError as e: 
            print(e) 

         
    def cleanup(self):  
        cv2.destroyAllWindows() 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("traffic_signal", anonymous=True)  
    TrafficSignal()  
