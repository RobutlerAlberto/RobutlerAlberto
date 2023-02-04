#!/usr/bin/env python3
# --------------------------------------------------
# Bruno, Fábio, Isabel 
# PSR, January 2023.
# RobutlerAlberto
# --------------------------------------------------

import cv2
import numpy as np
import rospy
from alberto_get_camera_footage import image
from std_msgs.msg import Int16
class ColorDetection:

    def __init__(self):
        # Calls the image class
        self.rgb_camera = image("/depth_camera/color/image_raw")
        
        # Set color goal
        self.color = ' '
        
        # Set object count
        self.objects_count = 0
        
        # Set detected contours
        self.detected_contours = set()
        
        # Set the minimum size of objects to detect
        self.min_contour_size = 500
        
        # Set a unique ID for each object
        self.object_id = 0
        
        # Create a dictionary to store the self.centroids of the objects and another to store the assigned IDs
        self.centroids = {}
        self.assigned_ids = {}
        
        # Define a maximum allowed distance between two self.centroids to consider them as the same object
        self.max_distance = 100
        
        # Listens for the mission ID
        self.mission_listener = rospy.Subscriber("/active_mission_ID", Int16, self.mission_listener_callback)
        
        # Image processing
        self.image_callback()

    def image_callback(self):

        #While theres no color the method should just wait
        while not self.color:
            pass
        
        while not rospy.is_shutdown():
            # Get color boundaries for the requested color
            lower_bound, upper_bound = self.color_chose()
            if 'cv_image' not in self.rgb_camera.image_args:
                continue
            img = self.rgb_camera.image_args['cv_image']
            
            # Convert the frame to HSV color space
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # Threshold the HSV image to get only the specified color
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(img,img, mask= mask)
                
            # Find contours in the resulting mask
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create a list to store the self.centroids of the current frame
            current_centroids = []
            self.detected_contours = []

            # Draw convex hulls around the detected objects
            for cnt in contours:
                if cv2.contourArea(cnt) < self.min_contour_size:
                    continue
                hull = cv2.convexHull(cnt)
                if cv2.contourArea(cnt) not in self.detected_contours:
                    self.detected_contours.append(cv2.contourArea(cnt))
                    # Uncomment to draw contours
                    # cv2.drawContours(img,[hull],0,(0,255,0),2) 
            
            # Compute the centroid of the current contour
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                current_centroids.append((cX, cY))
                
            
                # Draw a red circle around the center of the contour
                cv2.circle(img, (cX, cY), 3, (0, 0, 255), -1)

                # Assign a unique ID to the object if it's not in the dictionary
                if cv2.contourArea(cnt) not in self.assigned_ids:
                    self.object_id += 1
                    self.assigned_ids[cv2.contourArea(cnt)] = self.object_id

            # Loop over the stored self.centroids
            for key in self.centroids.keys():
                if key not in self.assigned_ids.values():
                    del self.centroids[key]
                    
            # Loop over the current self.centroids
            for i in range(len(current_centroids)):
                found = False
                
                for key in self.centroids.keys():
                    # Calculate the Euclidean distance between the current centroid and the stored self.centroids
                    distance = np.linalg.norm(np.array(current_centroids[i]) - np.array(self.centroids[key]))
                    
                    # Check if the distance is smaller than the maximum allowed distance
                    if distance < self.max_distance:
                        found = True
                        self.assigned_ids[self.detected_contours.pop()] = key
                        self.centroids[key] = current_centroids[i]
                        break

                if not found:
                    self.object_id += 1
                    self.assigned_ids[self.detected_contours.pop()] = self.object_id
                    self.centroids[self.object_id] = current_centroids[i]
            
            # Update the object count
            self.object_count = len(self.centroids)
            
            # Display the number of objects detected
            cv2.putText(img, "Objects detected: " + str(self.object_count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Display the resulting frame
            cv2.imshow("Result", img)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # Destroy all windows
        cv2.destroyAllWindows()

    def color_chose(self):
        # Define the color boundaries in the HSV color space for red
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])

        # Define the color boundaries in the HSV color space for green
        lower_green = np.array([40,50,50])
        upper_green = np.array([70,255,255])

        # Define the color boundaries in the HSV color space for blue
        lower_blue = np.array([100,50,50])
        upper_blue = np.array([140,255,255])

        # Define the color boundaries in the HSV color space for yellow
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([30,255,255])

        # Define the color boundaries in the HSV color space for orange
        lower_orange = np.array([5,50,50])
        upper_orange = np.array([15,255,255])

        # Define the color boundaries in the HSV color space for violet
        lower_violet = np.array([130, 50, 150])
        upper_violet = np.array([150, 255, 255])

        # Define bounds as the requested color
        if self.color == 'red':
            lower_bound = lower_red
            upper_bound = upper_red

        if self.color == 'green':
            lower_bound = lower_green
            upper_bound = upper_green

        if self.color == 'blue':
            lower_bound = lower_blue
            upper_bound = upper_blue

        if self.color == 'yellow':
            lower_bound = lower_yellow
            upper_bound = upper_yellow

        if self.color == 'orange':
            lower_bound = lower_orange
            upper_bound = upper_orange

        if self.color == 'violet':
            lower_bound = lower_violet
            upper_bound = upper_violet

        return lower_bound, upper_bound
        
    def mission_listener_callback(self, mission_id_msg):
        # Defines color as per the mission requirements to find violet balls
        if mission_id_msg.data == 20:
            self.color = 'violet'

        # Defines color as per the mission requirements to find blue cubes
        if mission_id_msg.data == 24:
            self.color = 'blue'


if __name__ == '__main__':
    rospy.init_node('color_detection', anonymous=False)
    color_detector = ColorDetection()
    # rospy.spin()  