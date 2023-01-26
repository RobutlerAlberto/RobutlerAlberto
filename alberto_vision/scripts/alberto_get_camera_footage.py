#!/usr/bin/env python3
# --------------------------------------------------
# Fábio, Isabel 
# PSR, January 2023.
# RobutlerAlberto
# --------------------------------------------------

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image:

    def __init__(self,camera_topic):
        
        # rospy.init_node('camera_footage', anonymous=False)
        # self.camera_topic = rospy.get_param("~camera_topic", default= "/rrbot/camera1/image_raw")
        

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.subscriberCallback)
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.subscriberCallback)
        self.image_args = {} # Contains cv_image
        self.begin_image = False

    

    def subscriberCallback(self,image_msg):

        self.begin_image = True # After receiving first image will be forever true

        #! If I assign directly to cv_image key, it will show flickers of the non flipped image 
        self.image_args["cv_image"] = self.bridge.imgmsg_to_cv2(image_msg,"bgr8") # Passthrough means wtv the encoding was before, it will be retained
        # rospy.loginfo("Received image message, image shape is " + str(self.image_args["cv_image"].shape))

    def showImage(self, processed):

        if not self.begin_image: # Only shows after first image is processed
            return

        
        cv2.imshow("CV_image", processed)

        pressed_key = cv2.waitKey(10) & 0xFF # To prevent NumLock issue

        if pressed_key == ord('q'):
            print("Quitting program")
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Order to quit") # Stops ros
            exit()                                 # Exits python script