#!/usr/bin/env python3

import rospy  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2  
import numpy as np
from std_msgs.msg import String 
from tensorflow import keras
from tensorflow.keras import layers
import tensorflow as tf


#This class will receive a ROS image and transform it to opencv format  
class TrafficSignalRec():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image 

        model = keras.models.load_model("signal_traffic_model.h5")

        ############################### SUBSCRIBERS #####################################  
        image_sub = rospy.Subscriber("traffic_signal_detection", Image, self.traffic_signal_detection_cb) 
        self.pub_traffic_signal = rospy.Publisher('traffic_signal_recognition', String, queue_size=1)  


        #********** INIT NODE **********###  
        r = rospy.Rate(50) #10Hz  
        while not rospy.is_shutdown():  
            if self.image_received:
                cv_image = self.cv_image
                #image_detected = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                #### Here we have to call the predictor of the CNN sending image_detected as input ####
                data=[]
                data.append(np.array(cv_image))
                X_test = np.array(data)
                
                pred_max = np.argmax(model.predict(X_test), axis=-1)
                pred = (model.predict(X_test))
                pred_per = pred[0][pred_max]
                print(pred)

                          
                if(pred_per >= 0.90):
                    if(pred_max == 0):
                        print("Stop")
                        self.pub_traffic_signal.publish("Stop")
                    if(pred_max == 1):
                        print("No Speed limit")
                        self.pub_traffic_signal.publish("No Speed limit")
                    if(pred_max == 2):
                        print("Turn right")
                        self.pub_traffic_signal.publish("Turn right")
                    if(pred_max == 3):
                        print("Ahead")
                        self.pub_traffic_signal.publish("Ahead")

                self.image_received = 0



            r.sleep()  

 

    def traffic_signal_detection_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format 
        try: 
            #print("received ROS image, I will convert it to opencv") 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="rgb8") 
            
            #self.cv_image = ros_numpy.numpify(ros_image)
            self.image_received = 1 #Turn the flag on 

        except CvBridgeError as e: 
            print(e) 

         
    def cleanup(self):  
        cv2.destroyAllWindows() 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("traffic_signal_recognition", anonymous=True)  
    TrafficSignalRec()  
