import numpy as np 
import logging
import argparse
import time 
import cv2 
import os 
import sys
if not os.path.exists('detections'):
    os.mkdir('detections')

# Initiate Logging 
logging.basicConfig(
    level=logging.INFO,
    format=f"%(asctime)s [%(levelname)s:] ->  %(message)s",
    handlers=[
        logging.FileHandler("info.log"),
        logging.StreamHandler(sys.stdout)
    ])

logging.info(msg="Start Session")

# Construct the argument parse and parse arguments
logging.info(msg="Define arguments")
ap  = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True, help="path to the folder that contains the images")
ap.add_argument("-y","--yolo", required=True, help="base path to yolo directory")
ap.add_argument("-c","--confidence", type=float, default=0.5, help='Minimum probabilty to filter weak detections')
ap.add_argument("-t","--threshold", type=float, default=0.3, help='Threshold when applying non-maximum supperssion')
args = vars(ap.parse_args())


def detect_image(args,image_path):
    # Load Yolo files (file.names, file.weights, file.cfg)

    logging.info(msg="Load files from yolo directory")
    labels_path  = os.path.sep.join([args['yolo'], "classes.names"])
    weights_path = os.path.sep.join([args['yolo'], "yolov4.weights"])
    config_path  = os.path.sep.join([args['yolo'], "yolov4.cfg"])


# Read file.names and defines our labels
    LABELS  = open(labels_path).read().strip().split('\n')

# Initialize a list of colors to represent each possible class label
    logging.info(msg='Define random colors for each class')
    np.random.seed(42)
    COLORS  = np.random.randint(0, 255, size=(len(LABELS), 3), dtype='uint8')


# Load our YOLO object detector
    logging.info(msg='Read CNN from weight and configuration files')
    net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

# Load out input image and grid it's dimension
    logging.info(msg="Load input image")
    image = cv2.imread(image_path)
    (H, W)= image.shape[:2]

# Determine output layer names 
    logging.info(msg='Get output layers names')
    layer_names = net.getLayerNames()
    layer_names = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# construct a blob from the input image and then perform a forward
    logging.info(msg='Construct Blob from input image')
    blob  =  cv2.dnn.blobFromImage(image, 1/255.0, (416,416), swapRB=True, crop=False)

# Give input image to Detector
    logging.info(msg='Pass image to Yolo object detector')
    net.setInput(blob)

# Analyze image through detector
    start = time.time()
    layerOutputs = net.forward(layer_names)
    end = time.time()
    logging.info(msg=f"Yolo took {end-start:.5f} seconds")


# initialize Lists of bounding boxes, confidenxes, and class IDs
    boxes = []
    confidences = []
    classIDs = []


# Loop over each of the layers outputs
    logging.info(msg="Start loop through Intersted Objects")
    for output in layerOutputs:
    # Loop over each of the detections
        for detection in output:
        #extract the class ID and confidence of the current Object detection
            scores     =  detection[5:]
            classID    =  np.argmax(scores)
            confidence =  scores[classID]

        # filter weak pridictions 
            if confidence > args['confidence']:
            # Scale the bounding box to size of image
                box = detection[:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype('int')
                x  = int(centerX - (width/2))
                y  = int(centerY - (height/2))

            # Update list of bounding box coordinates, confidances, and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
    
# apply non-maxima suppression to suppress weak, overlapping bounding box
    logging.info(msg='Apply Non-maxima supperssion to suppress bounding box')
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"], args["threshold"])

# Drow boxes in images
# ensure at least one detection exists
    logging.info(msg='Draw boxes in images')
    if len(idxs) > 0:
	# loop over the indexes we are keeping
     for i in idxs.flatten():
		# extract the bounding box coordinates
      (x, y) = (boxes[i][0], boxes[i][1])
      (w, h) = (boxes[i][2], boxes[i][3])
		# draw a bounding box rectangle and label on the image
      color = [int(c) for c in COLORS[classIDs[i]]]
      cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
      text = "{}: {:.2f}%".format(LABELS[classIDs[i]], confidences[i]*100)
      cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,1, color,3)
# show the output image

    outputPath = os.path.join("detections",os.path.basename(image_path))
    cv2.imwrite(outputPath, image)

for path, subdirs, files in os.walk(args["images"]):
    for name in files:
        image_path = os.path.join(path, name)
        detect_image(args,image_path)




