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



def is_plane(image):
    image = np.uint8(image)
    img_blur = cv2.GaussianBlur(image,(3,3), sigmaX=0, sigmaY=0)
    edges = cv2.Canny(image=img_blur, threshold1=0, threshold2=80)
    return cv2.countNonZero(edges) == 0


# ----------------------------------------------
# Initialization
# ----------------------------------------------

def main():
    
    rospy.init_node('camera_footage', anonymous=False)
    # Calls the image class
    rgb_camera = image("/depth_camera/color/image_raw")
    depth_map = image("/depth_camera/depth/image_raw")
    
    # Starts function with necessary args
    object_detection(rgb_camera, depth_map)

# ----------------------------------------------
# Execution
# ----------------------------------------------

def object_detection(rgb_camera, depth_map):

    # Load YOLO
    # Absolute path to files is needed
    rospack = rospkg.RosPack()
    path = rospack.get_path('alberto_vision') + r"/src"
    weight = path + r"/yolov3-tiny.weights"
    cfg = path + r"/yolov3-tiny.cfg"
    net = cv2.dnn.readNetFromDarknet(cfg, weight)

    #? This part detects objects and connects the identifiers to the object's bounding boxes
    classes = []

    with open(path + r"/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i- 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    while not rospy.is_shutdown():
        
        if 'cv_image' not in rgb_camera.image_args or 'cv_image' not in depth_map.image_args:
            continue

        # Stream read
        img = rgb_camera.image_args['cv_image']
        dm = depth_map.image_args['cv_image']
        # cv2.imwrite('depth_map.png', dm)

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

        # new_boxes = []
        for i in range(len(boxes)):

            if i in indexes:

                x, y, w, h = boxes[i]
                # interest_area = dm[y:h, x:w]

                dm = dm[~np.isnan(dm)]
                rospy.loginfo(dm)
                # dm = np.uint8(dm)
                # rospy.loginfo(dm)
                
                # if (dm.all() and not np.isnan(np.sum(dm))): # and not is_plane(interest_area)):
                #     rospy.loginfo(dm)

                    # label = str(classes[class_ids[i]])
                    # color = colors[i]
                    # cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                    # cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
        
        # ----------------------------------------------
        # Visualization
        # ----------------------------------------------

        # rgb_camera.showImage(img)



if __name__ == "__main__":
    main()
