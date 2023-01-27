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
import open3d as o3d

def is_plane(depth_map):
        # Create a point cloud from the depth map
        indices = np.column_stack(np.where(depth_map != 0))
        points = np.column_stack((indices, depth_map[np.where(depth_map != 0)]))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Estimate normals
        o3d.geometry.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # Get the normal vector of the point cloud
        normal_vector = pcd.normals[0]

        # Check if the normal vector is roughly constant
        if abs(normal_vector[0]) < 0.1 and abs(normal_vector[1]) < 0.1 and abs(normal_vector[2]-1) < 0.1:
            return True
        return False


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

        for i in range(len(boxes)):

            if i in indexes:

                x, y, w, h = boxes[i]

                # interest_area = dm[y:h, x:w]
                # rospy.loginfo(is_plane(interest_area))
                
                # if (interest_area.any()):
                #     depth_map.showImage(interest_area)

                label = str(classes[class_ids[i]])
                color = colors[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
        
        # ----------------------------------------------
        # Visualization
        # ----------------------------------------------

        rgb_camera.showImage(img)



if __name__ == "__main__":
    main()
