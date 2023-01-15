#!/usr/bin/env python3
# --------------------------------------------------
# Fábio Sousa.
# PSR, January 2023.
# RobutlerAlberto
# --------------------------------------------------

# Necessary imports
import cv2
import numpy as np

def main():

    # Load YOLO
    # Absolute path to files is needed
    path =r"/home/fabio/catkin_ws/src/RobutlerAlberto/alberto_vision/src" #! Change path to absolute path
    weight = path+r"/yolov3.weights"
    cfg = path+r"/yolov3.cfg"
    net = cv2.dnn.readNetFromDarknet(cfg, weight)

    classes = []

    with open("/home/fabio/catkin_ws/src/RobutlerAlberto/alberto_vision/src/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i- 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # Loading web cam
    cap = cv2.VideoCapture(0)

    while True:

        # Cap read
        _,img = cap.read()
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

        cv2.imshow("Image", img)

        if cv2.waitKey(1) == ord('q'):
            break
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

#* Failed attempt. Here is used imageai with pytorch, but there's an error I couldn't fix.
# from imageai.Detection import VideoObjectDetection
# import cv2

# def main():
#     obj_detect = VideoObjectDetection()
#     obj_detect.setModelTypeAsYOLOv3()
#     obj_detect.setModelPath('/home/fabio/catkin_ws/src/RobutlerAlberto/alberto_vision/scr/yolov3.pt')
#     obj_detect.loadModel()

#     cam_feed = cv2.VideoCapture(0)
#     cam_feed.set(cv2.CAP_PROP_FRAME_WIDTH, 650)
#     cam_feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 750)

#     while True:    
#         ret, img = cam_feed.read()   
#         annotated_image, preds = obj_detect.detectObjectsFromImage(input_image=img,
#                         input_type="array",
#                         output_type="array",
#                         display_percentage_probability=False,
#                         display_object_name=True)

#         cv2.imshow("", annotated_image)     
        
#         if (cv2.waitKey(1) & 0xFF == ord("q")) or (cv2.waitKey(1)==27):
#             break

#     cam_feed.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()