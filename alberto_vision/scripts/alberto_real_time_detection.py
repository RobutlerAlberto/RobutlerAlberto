#!/usr/bin/env python3
# --------------------------------------------------
# Fábio, Isabel
# PSR, January 2023.
# RobutlerAlberto
# --------------------------------------------------

# Necessary imports
import cv2
import rospkg
import rospy
import numpy as np
from alberto_get_camera_footage import image
from std_msgs.msg import Int16

class ObjectDetection:
    def is_plane(self,image):
        # Convert image
        image = np.uint8(image)
        # Blur for easier processing
        img_blur = cv2.GaussianBlur(image,(3,3), sigmaX=0, sigmaY=0)
        # Edge detection
        edges = cv2.Canny(image=img_blur, threshold1=0, threshold2=80)
        return cv2.countNonZero(edges) == 0


    # ----------------------------------------------
    # Initialization
    # ----------------------------------------------

    def __init__(self):
        # Listens for the mission ID
        rospy.Subscriber("/active_mission_ID", Int16, self.mission_listener_callback)


        rospy.init_node('camera_footage', anonymous=False)
        # Calls the image class
        self.rgb_camera = image("/depth_camera/color/image_raw")
        self.depth_map = image("/depth_camera/depth/image_raw")

        self.object = None
        self.object_2 = None

        self.found = False
        # Starts function with necessary args
        self.object_detection()

    # ----------------------------------------------
    # Execution
    # ----------------------------------------------

    def object_detection(self):

        # Load YOLO
        rospack = rospkg.RosPack()
        path = rospack.get_path('alberto_vision') + r"/src"
        weight = path + r"/yolov4-tiny.weights"
        cfg = path + r"/yolov4-tiny.cfg"
        net = cv2.dnn.readNetFromDarknet(cfg, weight)

        #? This part detects objects and connects the identifiers to the object's bounding boxes
        classes = []


        with open(path + r"/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]

        layer_names = net.getLayerNames()
        output_layers = [layer_names[i- 1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))

        while not rospy.is_shutdown():

            if 'cv_image' not in self.rgb_camera.image_args or 'cv_image' not in self.depth_map.image_args:
                continue

            # Stream read
            img = self.rgb_camera.image_args['cv_image']
            dm = self.depth_map.image_args['cv_image']
            
            if self.object == None:
                self.rgb_camera.showImage(img)
                continue
            # Get shape
            height, width, _ = img.shape

            # Detecting objects
            blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)

            # Showing informations on the screen
            class_ids = []
            confidences = []
            boxes = []

            for out in outs:

                for detection in out:

                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]

                    if confidence > 0.1:

                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

            font = cv2.FONT_HERSHEY_PLAIN

            for i in range(len(boxes)):

                if i in indexes:

                    x, y, w, h = boxes[i]
                    interest_area = dm[y:(y+h), x:(x+w)]

                    if interest_area.any() and not self.is_plane(interest_area):
                        label = str(classes[class_ids[i]])
                        color = colors[i]
                        self.found = False
                        if self.object_2:
                            if label == (self.object_2 or self.object):
                                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                                cv2.putText(img, self.object, (x, y + 30), font, 3, color, 3)
                                # print(self.object + ' found')
                        else:        
                            if label == self.object:
                                print('looking for sum bitches')
                                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                                cv2.putText(img, self.object, (x, y + 30), font, 3, color, 3)
                                # print(self.object + ' found')

            # ----------------------------------------------
            # Visualization
            # ----------------------------------------------
            self.rgb_camera.showImage(img)

    def mission_listener_callback(self, mission_id_msg):
            # Checks mission ID to detect computers
            if mission_id_msg.data == 26:
                self.object = 'laptop'
                self.object_2 = 'tv_monitor'
            
            # Checks mission ID to detect persons            
            if mission_id_msg.data == 24:
                self.object = 'person'
                self.object_2 = None

            if mission_id_msg.data not in (24,26):
                self.object = None
                self.object_2 = None

if __name__ == "__main__":
    ObjectDetection()