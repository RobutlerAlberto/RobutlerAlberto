#!/usr/bin/env python3
# --------------------------------------------------
# Bruno, Fábio, Isabel
# PSR, January 2023.
# RobutlerAlberto
# --------------------------------------------------

from copy import deepcopy
import cv2
import numpy as np
import rospy
from alberto_get_camera_footage import image
from std_msgs.msg import Int16,Bool
from colorama import Fore, Back, Style
from termcolor import colored
class ColorDetection:

    def __init__(self):
        # Calls the image class
        self.rgb_camera = image("/depth_camera/color/image_raw")

        # Set color goal
        self.color = ' '

        # Set variables to define is target must be/is round
        self.is_round = False
        self.must_be_round = False

        # Set variables to define is target must be/is quadrilateral
        self.is_square = False
        self.must_be_square = False

        # Set object count
        self.object_count = 0

        # Set default print color
        self.print_color = 'green'

        # Set variable to know if the text was already printed
        self.printed = False

        # Set detected contours
        self.detected_contours = set()

        # Set the found variable to know if an object has been found
        self.found = False

        # Set the minimum size of objects to detect
        self.min_contour_size = 550

        # Set a unique ID for each object
        self.object_id = 0

        # Create a dictionary to store the self.centroids of the objects and another to store the assigned IDs
        self.centroids = {}
        self.assigned_ids = {}

        # Define a maximum allowed distance between two self.centroids to consider them as the same object
        self.max_distance = 300

        # Define the color boundaries in the HSV color space for red
        self.lower_red = np.array([0,50,50])
        self.upper_red = np.array([10,255,255])

        # # Define the color boundaries in the HSV color space for green
        # self.lower_green = np.array([40,50,50])
        # self.upper_green = np.array([70,255,255])

        # Define the color boundaries in the HSV color space for blue
        self.lower_blue = np.array([100,50,50])
        self.upper_blue = np.array([140,255,255])

        # # Define the color boundaries in the HSV color space for yellow
        # self.lower_yellow = np.array([20,100,100])
        # self.upper_yellow = np.array([30,255,255])

        # Define the color boundaries in the HSV color space for orange
        # self.lower_orange = np.array([5,50,50])
        # self.upper_orange = np.array([15,255,255])

        # Define the color boundaries in the HSV color space for violet
        self.lower_violet = np.array([130, 50, 150])
        self.upper_violet = np.array([150, 255, 255])

        # Listens for the mission ID
        self.mission_listener = rospy.Subscriber("/active_mission_ID", Int16, self.mission_listener_callback)

        self.object_found_pub = rospy.Publisher("/object_found",Bool,queue_size=10)
        

        # Image processing
        self.image_processor()

    def object_found_callback(msg,data):
        pass

    def image_processor(self):
        # While ROS is running
        while not rospy.is_shutdown():
            # Continue if the image is empty
            if 'cv_image' not in self.rgb_camera.image_args:
                continue

            # Get image
            img = self.rgb_camera.image_args['cv_image']

            # Create working copy
            img_gui = deepcopy(img)

            # If there's no color, continue showing image
            if not self.color:
                cv2.imshow("Color Detection", img)
                cv2.waitKey(1)
                continue

            # Create window
            cv2.namedWindow('Color Detection', cv2.WINDOW_AUTOSIZE)
            cv2.startWindowThread()

            # Get color boundaries for the requested color
            lower_bound, upper_bound = self.color_chose()

            # Continue if the bounds are empty
            if np.logical_and(lower_bound, upper_bound) is None:
                continue

            # Convert the frame to HSV color space
            hsv = cv2.cvtColor(img_gui, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only the specified color
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(img_gui,img_gui, mask= mask)

            # Find contours in the resulting mask
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Create a list to store the self.centroids of the current frame
            current_centroids = []
            self.detected_contours = []

            # Draw convex hulls around the detected objects
            for cnt in contours:
                # Get contour area
                contour_area = cv2.contourArea(cnt)

                # Check if the area has the minimum size
                if contour_area < self.min_contour_size:
                    continue
                
                # Check is object must be round
                if self.must_be_round:
                    # Fit a circle around the contour
                    (xs, ys), radius_s = cv2.minEnclosingCircle(cnt)
                    center_s = (int(xs), int(ys))
                    radius_s = int(radius_s)

                    # Calculate the area of the circle
                    circle_area = np.pi * radius_s**2

                    # Calculate the ratio of the two areas
                    ratio_s = contour_area / circle_area

                    # If the ratio is close to 1, consider the object to be round
                    if 0.8 < ratio_s < 1.2:
                        self.is_round = True
                    else:
                        continue
                
                # Approximate contour 
                approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt, True), True)
                # Check if the object must be a square
                if self.must_be_square:
                    # If the approximate contour has at least 4 vertices
                    if len(approx) in range(4, 7):
                        # Fit a rectangle around the contour
                        x_r, y_r, w_r, h_r = cv2.boundingRect(cnt)
                        # Calculate the ratio of the areas
                        ratio_r= float(w_r)/h_r
                        # If the ratio is close to 1, consider the object as a rectangle
                        if ratio_r>=0.90 and ratio_r<=1.1:
                            self.is_square = True
                    else:
                        continue
                hull = cv2.convexHull(cnt)

                if contour_area not in self.detected_contours:
                    self.detected_contours.append(contour_area)
                    # Uncomment to draw contours
                    # cv2.drawContours(img_gui,[hull],0,(0,255,0),2)


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
                cv2.circle(img_gui, (cX, cY), 3, (0, 0, 255), -1)

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
                self.found = False

                for key in self.centroids.keys():
                    # Calculate the Euclidean distance between the current centroid and the stored self.centroids
                    distance = np.linalg.norm(np.array(current_centroids[i]) - np.array(self.centroids[key]))

                    # Check if the distance is smaller than the maximum allowed distance
                    if distance < self.max_distance:
                        self.found = True
                        self.assigned_ids[self.detected_contours.pop()] = key
                        self.centroids[key] = current_centroids[i]
                        break

                
                if not self.found:
                    self.printed = False
                    self.object_id += 1
                    self.assigned_ids[self.detected_contours.pop()] = self.object_id
                    self.centroids[self.object_id] = current_centroids[i]

            # Update the object count
            self.total_object_count = len(self.centroids)
            self.current_object_count = len(current_centroids)


            # Display the number of objects detected
            cv2.putText(img_gui, "Total objects detected: " + str(self.total_object_count), (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(img_gui, "Current objects detected: " + str(self.current_object_count), (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 175), 2)

            # Display the resulting frame
            cv2.imshow("Color Detection", img_gui)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            
            # Print if object is detected
            for i in range (self.current_object_count):
                if self.found == True and not self.printed:
                    if self.is_round:
                        obj = 'Sphere'
                    if self.is_square:
                        obj = 'Cube'
                    print(Style.BRIGHT + Back.BLACK + Fore.GREEN + obj + ' of color ' + colored(self.color, self.print_color) + Fore.GREEN + Style.BRIGHT + Back.BLACK + ' was detected' + Style.RESET_ALL)
                    self.printed = True

                    found_msg = Bool()
                    found_msg.data = self.found
                    self.object_found_pub.publish(found_msg)
        
        # Destroy all windows
        cv2.destroyAllWindows()

    def color_chose(self):
        # Sets the color boundaries
        lower_bound = None
        upper_bound = None

        # Define boundaries for the requested color
        if self.color == 'red':
            lower_bound = self.lower_red
            upper_bound = self.upper_red

        # if self.color == 'green':
        #     lower_bound = self.lower_green
        #     upper_bound = self.upper_green

        if self.color == 'blue':
            lower_bound = self.lower_blue
            upper_bound = self.upper_blue

        # if self.color == 'yellow':
        #     lower_bound = self.lower_yellow
        #     upper_bound = self.upper_yellow

        # if self.color == 'orange':
        #     lower_bound = self.lower_orange
        #     upper_bound = self.upper_orange

        if self.color == 'violet':
            lower_bound = self.lower_violet
            upper_bound = self.upper_violet

        return lower_bound, upper_bound

    def mission_listener_callback(self, mission_id_msg):
        # Defines color as per the mission requirements to find violet balls
        if mission_id_msg.data == 20:
            self.color = 'violet'
            self.must_be_round = True
            self.print_color = 'magenta'
            self.centroids =  {}
            self.printed = False

        # Defines color as per the mission requirements to find blue cubes
        if mission_id_msg.data == 24:
            self.must_be_square = True
            self.color = 'blue'
            self.print_color = 'blue'
            self.centroids =  {}
            self.printed = False

        # Defines color as per the mission requirements to find red balls
        if mission_id_msg.data == 25:
            self.must_be_round = True
            self.color = 'red'
            self.print_color = 'red'
            self.centroids =  {}
            self.printed = False

        # If the mission ID it not one relevant to this code, reset variables
        if mission_id_msg.data not in (20, 24, 25):
            self.color = None
            self.centroids =  {}
            self.printed = False

if __name__ == '__main__':
    rospy.init_node('color_detection', anonymous=False)
    color_detector = ColorDetection()