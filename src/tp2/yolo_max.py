# USAGE
# python real_time_object_detection.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel

# import the necessary packages
from PIL import Image
from mss import mss
import numpy as np
import argparse
import imutils
import cv2
import sys
import glob
import os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time

IM_WIDTH = 800
IM_HEIGTH = 600

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
#ap.add_argument("-y", "--yolo", required=True,
#    help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.5, #0.9
    help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.4, #0.6
    help="threshold when applyong non-maxima suppression")
args = vars(ap.parse_args())

# load the COCO class labels our YOLO model was trained on
args["yolo"] = "yolo-coco"
labelsPath = os.path.sep.join([args["yolo"], "coco.names"])
LABELS = open(labelsPath).read().strip().split("\n")

# initialize a list of colors to represent each possible class label
np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
    dtype="uint8")

# derive the paths to the YOLO weights and model configuration
weightsPath = os.path.sep.join([args["yolo"], "yolov3.weights"])
configPath = os.path.sep.join([args["yolo"], "yolov3.cfg"])

# load our YOLO object detector trained on COCO dataset (80 classes)
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

def arrayofdepthcamera(imagedepth):
    global depthimage
    depthimage = imagedepth

def yoloVariables(x, y, z):
    global objCoordinates
    global objConfidence
    global objDetectionName 
    objCoordinates = x
    objConfidence = y
    objDetectionName = z

def objSum(number):
    global objNumber
    objNumber = number

def distance(number1):
    global objDistance
    objDistance = number1

#def bwframe(bwframe):
    #global frame23
    #frame23 = bwframe

def count(a):
    global counter
    counter =a

def process1(b):
    global step1
    step1 = b

def process2(c):
    global step2
    step2 = c

def process3(d):
    global step3
    step3 = d

def getdepth(array):
    if type(array) == int:
        distance(0)
    else:
        yhalf = array[0] + (array[2] * 0.5)
        xhalf = array[1] + (array[3] * 0.5)

        pixel = depthimage[int(xhalf),int(yhalf)]
        normalized = (pixel[2] + pixel[1] * 256 + pixel[0] * 256 * 256) / (256 * 256 * 256 - 1)
        in_meters = 1000 * normalized
        distance(in_meters)

def myprocess(image):

    #Configurate the RGB-Image and starting the yolo-function
    array = np.array(image.raw_data)
    i2 = array.reshape(IM_HEIGTH,IM_WIDTH,4)
    frame = i2[:,:,:3]
    yolostart(frame)

    return frame 

def yolostart(imagevariable):
    if step1 > 1:
        process1(0)
        frame = imagevariable
        (H, W) = imagevariable.shape[:2]
        
        # determine only the *output* layer names that we need from YOLO
        #ln = net.getLayerNames()
        #ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        ln = net.getUnconnectedOutLayersNames()

        # construct a blob from the input image and then perform a forward
        # pass of the YOLO object detector, giving us our bounding boxes and
        # associated probabilities
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416),
            swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()
        
        # show timing information on YOLO
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))

        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = [] 

        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > args["confidence"]:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)


        if counter > 0 :            
            # apply non-maxima suppression to suppress weak, overlapping bounding
            # boxes
            idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
                args["threshold"])

            j = [0]*900
            s = [""]*900
            k = [0]*900
            objSum(len(idxs))

            # ensure at least one detection exists
            if len(idxs) > 0 & len(idxs) < 100:

                # loop over the indexes we are keeping
                for i in idxs.flatten():
                    
                    if i < 10:
                        # extract the bounding box coordinates
                        (x, y) = (boxes[i][0], boxes[i][1])
                        (w, h) = (boxes[i][2], boxes[i][3])

                        s[i] = [LABELS[classIDs[i]]]

                        j[i] = [boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]]

                        k[i]=[confidences[i]]
                        yoloVariables(j, k, s)

                        

                        process2(1)

            else:
                print("none")


        count(2)        

    process1(2)
    return None  

def myprocess2(image2):
    #Giving deept-Image to depth-function
    depthAlgo(image2)
    return None

def depthAlgo(image2):
    if step2 == 1:

        array = np.array(image2.raw_data)
        i = array.reshape(IM_HEIGTH,IM_WIDTH,4)
        i2 = i[:,:,:3]
        #pixel = i2[398,344]
        arrayofdepthcamera(i2)


            
        
        
        #normalized = (pixel[2] + pixel[1] * 256 + pixel[0] * 256 * 256) / (256 * 256 * 256 - 1)
        #in_meters = 1000 * normalized
        #distance(in_meters)


        # Entfernung aus GBR (nicht RGB!) Daten berechnen
        

        process2(0)
        process3(1)
    return None    

actor_list = []

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Blueprints erstellen
    # Blueprints = Fahrzeuge und Fußgänger
    blueprint_library = world.get_blueprint_library()
    #bp_1 = random.choice(blueprint_library.filter('vehicle'))
    bp_1 = blueprint_library.filter('low_rider')[0]
    bp_2 = blueprint_library.filter('model3')[0]

    if bp_1.has_attribute('color'):
            color = random.choice(bp_1.get_attribute('color').recommended_values)
            bp_1.set_attribute('color', color)
    
    if bp_2.has_attribute('color'):
            color = random.choice(bp_2.get_attribute('color').recommended_values)
            bp_2.set_attribute('color', color)
    

    # Spwan Points festlegen und Fahrzeugeigenschaften festlegen
    # Koordination durch Probieren
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    spawn_point_bp_1 = carla.Transform(carla.Location(x=130,y=-6,z=1), carla.Rotation(yaw=180))
    spawn_point_bp_2 = carla.Transform(carla.Location(x=150,y=-6,z=1), carla.Rotation(yaw=180))
    spawn_point_bp_3 = carla.Transform(carla.Location(x=120,y=-3.5,z=1), carla.Rotation(yaw=180))
    vehicle_1 = world.spawn_actor(bp_1, spawn_point_bp_1)
    vehicle_2 = world.spawn_actor(bp_2, spawn_point_bp_2)
    vehicle_3 = world.spawn_actor(bp_2, spawn_point_bp_3)
    #vehicle_1.set_autopilot(True)
    vehicle_1.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    vehicle_2.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))
    vehicle_3.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))
    actor_list.append(vehicle_1)
    actor_list.append(vehicle_2)
    actor_list.append(vehicle_3)

    cam_bp_2 = blueprint_library.find('sensor.camera.rgb') #rgb
    #cam_bp_2.set_attribute('fov', '110')
    cam_bp_2.set_attribute('image_size_x', '800')
    cam_bp_2.set_attribute('image_size_y', '600')
    cam_bp_2.set_attribute('sensor_tick', '0.5')
    transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    sensor = world.spawn_actor(cam_bp_2, transform, attach_to=vehicle_2)

    cam_bp_3 = blueprint_library.find('sensor.camera.depth') #rgb
    #cam_bp_2.set_attribute('fov', '110')
    cam_bp_3.set_attribute('image_size_x', '800')
    cam_bp_3.set_attribute('image_size_y', '600')
    cam_bp_3.set_attribute('sensor_tick', '0.5')
    transform2 = carla.Transform(carla.Location(x=1.5, z=2.4))
    sensor2 = world.spawn_actor(cam_bp_3, transform2, attach_to=vehicle_2)
  

    #inits  
    count(0)
    process1(2)
    process2(0)
    process3(0)
    #yolo 
    sensor.listen(lambda image: (myprocess(image)))
    #FirstYolorun for Hotfix
    count(1)
    #depth
    sensor2.listen(lambda image2: (myprocess2(image2)))   


    #temporaly output
    while True:
        if step3 == 1:
            if counter > 1:
                for i in range(objNumber):
                    getdepth(objCoordinates[i])
                    print(objDetectionName[i], "at", objCoordinates[i], "Distance:", objDistance, "Confidence", objConfidence[i])

                process3(0)

    time.sleep(55)


#objNumber
#Anzahl der detek. obj

#pixelcoordinaten
#objCoordinates[i] ganze Zeile, objCoordinates[i][0]
#i   0 1 2 3
#1   x y w h
#2   x y w h

#Objektname
#i name
#1 car
#2 car 
#3 truck

#Abstand
#variable -> objDistance (bisher nur ein fester Integerwert in m)

#Confidence (Yologenauigkeit)
#0.99...
#0.83...
#0.73...

finally:
    for actor in actor_list:
        actor.destroy()
        print("All cleaned up!")


