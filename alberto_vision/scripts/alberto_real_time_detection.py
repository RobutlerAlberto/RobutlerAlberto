#!/usr/bin/env python3
# --------------------------------------------------
# Fábio, Isabel 
# PSR, January 2023.
# RobutlerAlberto
# --------------------------------------------------

# Necessary imports
import cv2
import numpy as np
from alberto_get_camera_footage import image

# ----------------------------------------------
# Initialization
# ----------------------------------------------

def main():
    
    #Calls the image class
    bottom_front_camera = image()      
    
    # Starts function with necessary args
    object_detection(bottom_front_camera.image_args, bottom_front_camera)

# ----------------------------------------------
# Execution
# ----------------------------------------------

def object_detection(alberto_camera, bottom_front_camera):

    # Load YOLO
    # Absolute path to files is needed
    path =r"/home/fabio/catkin_ws/src/RobutlerAlberto/alberto_vision/src" #! Change path to absolute path
    weight = path+r"/yolov3-tiny.weights"
    cfg = path+r"/yolov3-tiny.cfg"
    net = cv2.dnn.readNetFromDarknet(cfg, weight)

    #? This part detects objects and connects the identifiers to the object's bounding boxes
    classes = []

    with open("/home/fabio/catkin_ws/src/RobutlerAlberto/alberto_vision/src/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i- 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    while True:
        
        # Stream read
        img = alberto_camera['cv_image']

        # Get shape
        height, width, channels = img.shape

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

                if confidence > 0.5:

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
                label = str(classes[class_ids[i]])
                color = colors[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
        
# ----------------------------------------------
# Visualization
# ----------------------------------------------

        bottom_front_camera.showImage(img)



if __name__ == "__main__":
    main()