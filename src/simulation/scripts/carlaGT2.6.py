#!/usr/bin/env python3.5

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
	Example of automatic vehicle control from client side.
"""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref
import rospy 
import numpy

import glob 
import cv2
import imutils
import time
from mss import mss
import numpy as np

from object_list.msg import ObjectsList
from object_list.msg import ObjectList
from object_list.msg import Geometric as geo

from pyimagesearch.centroidtracker import CentroidTracker
from imutils.video import VideoStream


try:
	import pygame
	from pygame.locals import KMOD_CTRL
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_q
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
	import numpy as np
except ImportError:
	raise RuntimeError(
		'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass

# ==============================================================================
# -- add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
	sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
	pass

import carla
from carla import ColorConverter as cc
from agents.navigation.roaming_agent import RoamingAgent
from agents.navigation.basic_agent import BasicAgent


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def find_weather_presets():
	rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
	name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
	presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
	return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
	name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
	return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

IM_WIDTH = 800
IM_HEIGTH = 600


CONFIDENCE = 0.7

THRESHOLD = 0.6

# load the COCO class labels our YOLO model was trained on
##############args["yolo"] = "yolo-coco"
labelsPath = "/opt/yolofolders/yolo-coco/coco.names"
LABELS = open(labelsPath).read().strip().split("\n")

# initialize a list of colors to represent each possible class label
np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")

# derive the paths to the YOLO weights and model configuration
weightsPath = "/opt/yolofolders/yolo-coco/yolov3.weights"
configPath = "/opt/yolofolders/yolo-coco/yolov3.cfg"

# load our YOLO object detector trained on COCO dataset (80 classes)
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

def arrayofdepthcamera(imagedepth):
	global depthimage
	depthimage = imagedepth

def yoloVariables(x, y, z, iD_var):
	global objCoordinates
	global objConfidence
	global objDetectionName
	global objID_Var 
	objCoordinates = x
	objConfidence = y
	objDetectionName = z
	objID_Var = iD_var

def objSum(number):
	global objNumber
	objNumber = number

def length_def(transform_length):
	global objLength
	objLength = transform_length

def distance(dist):
	global objDistance
	objDistance = dist

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

def setTime(zeit):
	global TIME_PERIOD
	TIME_PERIOD = zeit 

#=================================

####################################################
### Klassen für Berechnung der Objektliste #########
####################################################
CAM_ANGULAR = 90 #90 degree camera view , const.
CAM_XSIZE = 800  #x-Size Pixel
CAM_YSize = 600  #y-Size Pixel
PI = 3.14159265359
#=================================
class GeometricMSG:
	#========= Previous Values========
	xI=0 #Positions
	yI=0
	vxI=0 #Velocity
	vyI=0
	yawI=0 #Angel
	x = 0
	y = 0
	vx= 0 
	vy= 0 
	ax= 0 
	ay= 0
	yaw = 0
	yawrate = 0

	def __init__(self): #direct Distance, x-Coordinate(Pixel),y-Coordinate
		self.x = 0
		self.y = 0
		self.vx= 0 
		self.vy= 0 
		self.ax= 0 
		self.ay= 0
		self.yaw = 0
		self.yawrate = 0
		
		
	def setValue(self,dirDist, array):
		xP = array[1]
		yP = array[0]
		#print("altes x")
		#print(self.x)
		xI = self.x         ################################
		#print("Neues xI")
		#print(xI)
		yI = self.y         #
		vxI = self.vx       # Müssen vllt alle global
		vyI = self.vy       #
		yawI = self.yaw     ################################
		#===== Proportionality
		yhelp = yP - CAM_XSIZE/2
		Faktor = yhelp/400
		Winkel = Faktor * 45  
		#===== Positions
		# print(dirDist)


		self.y = dirDist * math.sin(Winkel*PI/180)
		
		self.x = dirDist * math.cos(Winkel*PI/180)
		#print("Berechnetes x")
		#print(self.x)
		#===== Velocity
		if xI == 0:
			self.vx = 0
		else:
			self.vx = (self.x - xI)/TIME_PERIOD 

		if yI == 0:
			self.vy = 0
		else:
			self.vy = (self.y - yI)/TIME_PERIOD
		#===== Acceleration
		if vxI == 0:
			self.ax = 0
		else: 
			self.ax = (vxI - self.vx)/TIME_PERIOD

		if vyI == 0:
			self.ay = 0
		else: 
			self.ay = (vyI - self.vy)/TIME_PERIOD
		#====== Angel
		if (xI == 0 and yI ==0 ):
			self.yaw = 0
		else:
			#print("Geschwindigkeiten vx,vy")
			#print(self.vx, self.vy)
			self.yaw = math.asin(self.vy/math.sqrt(self.vx*self.vx+self.vy*self.vy)) * 180/PI
			#print("yaw:")
			#print(self.yaw)
			#print("Argument in sin:")
			#print(self.vy/math.sqrt(self.vx*self.vx+self.vy*self.vy))
		#======= Angelvelocity
		if yawI == 0:
			self.yawrate = 0
		else:
			self.yawrate = (self.yaw - yawI)/TIME_PERIOD
			#print("yawrate")
			#print(self.yawrate)
		#print("x,y, vx,vy")
		#print(self.x, self.y, self.vx, self.vy)
		#print("Beschleunigung")
		#print(self.ax, self.ay)

class DimensionMSG:
	width = 0
	length = 0
	height = 0
	def __init__(self): #dirDist: direct Distance , xP: x-Coordinates (Pixel), hP: heigth , wP: width, className: KlassifizierungObjekt
		self.width = 0
		self.length= 0
		self.height= 0

	def setDimension(self,dirDist,array,className, objLeng):
		#==== Adjust Scale
		classNameString =""
		classNameString = classNameString.join(className)
		yP = array[0]
		wP = array[2]
		hP = array[3]
		yhelp = yP - CAM_XSIZE/2
		Faktor = yhelp/400
		Winkel = Faktor * 45
		yMeter = dirDist * math.sin(Winkel*PI/180)
		if yhelp != 0:
			meterPerPixel = yMeter/yhelp
		else:
			return 1

		
		#self.height = hP * meterPerPixel
		if classNameString == 'car':
			self.length = objLeng
			self.width = wP * meterPerPixel
			self.height = hP * meterPerPixel
		elif classNameString == 'truck':
			self.length = objLeng
			self.width = wP * meterPerPixel
			self.height = hP * meterPerPixel
		elif classNameString == 'motorbike':
			self.length = objLeng
			self.width = wP * meterPerPixel
			self.height = hP * meterPerPixel
		elif classNameString == 'bicycle':
			self.length = objLeng
			self.width = wP * meterPerPixel
			self.height = hP * meterPerPixel
		elif classNameString == 'person':
			self.length = objLeng
			self.width = wP * meterPerPixel
			self.height = hP * meterPerPixel
		elif classNameString == 'stacionary':
			self.length = objLeng
			self.width = wP * meterPerPixel
			self.height = hP * meterPerPixel
		else:
			self.length = 0
			self.width = 0
			self.height = 0
		#print(self.length)
		#print(classNameString)

class ClassificationMSG:
	car = 0
	truck = 0
	motorcycle = 0
	bicycle = 0
	pedestrian = 0
	stacionary = 0
	other = 0

	def __init__(self):
		self.car = 0
		self.truck = 0
		self.motorcycle = 0
		self.bicycle = 0
		self.pedestrian = 0
		self.stacionary = 0
		self.other = 0

	def setClass(self,className):
		classNameString = ""
		classNameString = classNameString.join(className)
		if classNameString == 'car':
			self.car = 1
			self.truck = 0
			self.motorcycle = 0
			self.bicycle = 0
			self.pedestrian = 0
			self.stacionary = 0
			self.other = 0
		elif classNameString == 'truck':
			self.truck = 1
			self.car = 0
			self.motorcycle = 0
			self.bicycle = 0
			self.pedestrian = 0
			self.stacionary = 0
			self.other = 0
		elif classNameString == 'motorbike':
			self.motorcycle = 1
			self.car = 0
			self.truck = 0
			self.bicycle = 0
			self.pedestrian = 0
			self.stacionary = 0
			self.other = 0
		elif classNameString == 'bicycle':
			self.bicycle = 1
			self.car = 0
			self.truck = 0
			self.motorcycle = 0
			self.pedestrian = 0
			self.stacionary = 0
			self.other = 0
		elif classNameString == 'person':
			self.pedestrian = 1
			self.car = 0
			self.truck = 0
			self.motorcycle = 0
			self.bicycle = 0
			self.stacionary = 0
			self.other = 0
		elif classNameString == 'stacionary':
			self.stacionary = 1
			self.car = 0
			self.truck = 0
			self.motorcycle = 0
			self.bicycle = 0
			self.pedestrian = 0
			self.other = 0
		else:
			self.car = 0
			self.truck = 0
			self.motorcycle = 0
			self.bicycle = 0
			self.pedestrian = 0
			self.stacionary = 0
			self.other = 1

#================ Not available at the moment
class FeatureMSG:
	FL = 0
	FM = 0
	FR = 0
	MR = 0
	RR = 0
	RM = 0
	RL = 0
	ML = 0

	def __init__(self):
		self.FL = 0
		self.FM = 0
		self.FR = 0
		self.MR = 0
		self.RR = 0
		self.RM = 0
		self.RL = 0
		self.ML = 0

class AllMSG:
	geo = None
	dim = None
	klas = None
	id_Klasse = ""
	iD_Classname = ""
	stillChanging = False
	confidence_spez = 0
	dist_ = 0
	coordinates_ = 0

	def __init__(self):
		self.geo = GeometricMSG()
		self.dim = DimensionMSG()
		self.klas = ClassificationMSG()

	def setAllValues(self,dirDist,objKoordinaten, className,iD0,confi,objLeng):
		self.geo.setValue(dirDist,objKoordinaten)
		self.dim.setDimension(dirDist,objKoordinaten,className,objLeng)
		self.klas.setClass(className)
		self.id_Klasse = iD0[0]
		self.confidence_spez = confi
		#print(self.id_Klasse)
		#print(type(self.id_Klasse))
		self.iD_Classname = self.iD_Classname.join(className)
		self.stillChanging = True
		self.dist_ = dirDist
		self.coordinates_ = objKoordinaten

	def changeTrigger(self):
		self.stillChanging = False 
		


	def getGeo():
		return self.geo

	def getDim():
		return self.dim

	def getCls():
		return self.klas



def checkDetectObj(className):
	classNameString = ""
	classNameString = classNameString.join(className)
	if classNameString == 'car':
		return 1
	elif classNameString == 'truck':
		return 1
	elif classNameString == 'motorbike':
		return 1
	elif classNameString == 'bicycle':
		return 1
	elif classNameString == 'person':
		return 1
	elif classNameString == 'stacionary':
		return 1
	else:
		return 0


	

####################################################
### Ende Klassen fuer Objektliste  #################
####################################################

####################################################
### Initialisierung Node, publishing ###############
####################################################
def yoloTalker(allData):
	#========= Node and Topic init=====================
	pub = rospy.Publisher('camera_calculation',ObjectsList, queue_size=100) #initialisierung topic'calculated' msg'ObjectsList' MsgCacha'queuesize'
	rospy.init_node('camera', anonymous=False) #initialisierung node'vidCalcNode' Zahlennamenserweiterung'anonymous'
	#no rate=rospy.Rate(50) because of yolo intern rate

	# no while loop for node.shutdown rospy.is_shutdown() because yolo calls yoloTalker every time

	#========== Init ObjectsList (possibly [GenericList] or static Objectlist==
	b = ObjectsList()
	#a_car1 = ObjectList()
	#a_car1.obj_id = allData1.id_Klasse
	#a_car2 = ObjectList()
	#a_car2.obj_id = allData2.id_Klasse
	#a_car3 = ObjectList()
	#a_car3.obj_id = allData3.id_Klasse

	for i in range(len(allData)):
		if not (allData[i].id_Klasse or allData[i].id_Klasse ==0):
			print("---------------------------------------------")
			continue
		b.obj_list.append(ObjectList())
		b.obj_list[i].obj_id = allData[i].id_Klasse
		b.obj_list[i].geometric.x = allData[i].geo.x
		b.obj_list[i].geometric.y = allData[i].geo.y
		b.obj_list[i].geometric.vx = allData[i].geo.vx
		b.obj_list[i].geometric.vy = allData[i].geo.vy
		b.obj_list[i].geometric.ax = allData[i].geo.ax
		b.obj_list[i].geometric.ay = allData[i].geo.ay
		b.obj_list[i].geometric.yaw = allData[i].geo.yaw
		b.obj_list[i].geometric.yawrate = allData[i].geo.yawrate

		b.obj_list[i].dimension.length = allData[i].dim.length
		b.obj_list[i].dimension.width = allData[i].dim.width
		b.obj_list[i].dimension.height = allData[i].dim.height
 

		b.obj_list[i].classification.car = allData[i].klas.car
		b.obj_list[i].classification.truck = allData[i].klas.truck
		b.obj_list[i].classification.motorcycle = allData[i].klas.motorcycle
		b.obj_list[i].classification.bicycle = allData[i].klas.bicycle
		b.obj_list[i].classification.pedestrian = allData[i].klas.pedestrian
		b.obj_list[i].classification.stacionary = allData[i].klas.stacionary
		b.obj_list[i].classification.other = allData[i].klas.other

		b.obj_list[i].prop_existence = allData[i].confidence_spez

		##Second message check
	#for j in range(len(allData)):
	#	if not (allData[j].id_Klasse or allData[j].id_Klasse ==0):
	#		print("Hier kommen die ObjektIDs")
	#		print(allData[j].id_Klasse)




	#========== appending ObjectsList ==================
	b.header.frame_id = "ObjectsList VideoGenerated"
	b.header.stamp = rospy.Time.now()


	#========== just append if object is detected
	#if a_car1.geometric.x != 0 and a_car1.geometric.y != 0 and count>=1:
	#	b.obj_list.append(a_car1)
	#if a_car2.geometric.x != 0 and a_car2.geometric.y != 0 and count>=2:
	#	b.obj_list.append(a_car2)

	#if a_car3.geometric.x != 0 and a_car3.geometric.y != 0:
	#if a_car3.geometric.x != 0 and a_car3.geometric.y != 0 and count>=3:
	#	b.obj_list.append(a_car3)
	if not b:
		print("List is empty")
		return 1



	#========== Publishing and printing in terminal=====
	pub.publish(b)
	rospy.loginfo(b)


####################################################
### Ende Init und Publishing #######################
####################################################
########
ct = CentroidTracker()
########

def getlength(arraycoordinates,transform_name, transform_len):
	array = arraycoordinates
	newimageRGB = rgb_frame
	depth = depthimage
	if type(array) == int:
		length_def(0)
	elif 0>array[0]>IM_WIDTH and 0>array[1]>IM_HEIGTH and array[2]<0 and array[3]<0:
		length_def(0)
	elif transform_len>3:
		length_def(0)
	elif 0<array[0]<IM_WIDTH and 0<array[1]<IM_HEIGTH and array[2]>0 and array[3]>0:
		frame_of_interest = newimageRGB[array[1]:(array[1]+array[3]),array[0]:(array[0]+array[2])]
		frame_of_interest_grayscaled = cv2.cvtColor(frame_of_interest, cv2.COLOR_BGR2GRAY)
		gaus_threshold = cv2.adaptiveThreshold(frame_of_interest_grayscaled, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115,1)
		

		nearest = 1000
		furthest = 0

		#######################check = False

		for i in range(array[3]):
			for j in range(array[2]):
				if gaus_threshold[i,j] == 0 and (array[0]+i)<IM_HEIGTH and (array[1]+j)<IM_WIDTH:
					pixel_length = depth[(array[1]+i),(array[0]+j)]
					calc = (pixel_length[2] + pixel_length[1] * 256 + pixel_length[0] * 256 * 256) / (256 * 256 * 256 - 1)
					calc_in_meters = 1000 * calc


					if calc_in_meters < nearest:
						nearest = calc_in_meters
					if calc_in_meters > furthest:
						furthest = calc_in_meters
				else:
					break

		length = furthest-nearest
		if length == -1000:
			length_def(0)
		elif transform_name == ['car'] and length > 7:
			length_def(4.0)
		elif transform_name == ['person'] and length > 2:
			length_def(0.6)
		else:
			length_def(length)
		
	else:
		length_def(0)



def getdistance(arraycoordinates,transform_len, winkelarray):
	winkel = winkelarray
	array = arraycoordinates
	depth = depthimage
	length = objLength
	if type(array) == int:
		distance(0)
	elif 0>array[0]>IM_WIDTH and 0>array[1]>IM_HEIGTH and array[2]<0 and array[3]<0:
		distance(0)
	elif transform_len>3:
		distance(0)
	elif 0<array[0]<IM_WIDTH and 0<array[1]<IM_HEIGTH and array[2]>0 and array[3]>0:
		ycenter = array[0] + (array[2] * 0.35)
		xcenter = array[1] + (array[3] * 0.5)

		pixel = depth[int(xcenter),int(ycenter)]
		normalized = (pixel[2] + pixel[1] * 256 + pixel[0] * 256 * 256) / (256 * 256 * 256 - 1)
		in_meters = (1000 * normalized) #+ (length * 0.5)
		hlength = length * 0.5
		if winkel != [] and winkel != 0:# and type(winkel) == float:
			finaldistance = (hlength**2 + in_meters**2 - (2 * hlength * in_meters * math.cos(float(winkel[0]))))**(1/2)
		else:
			finaldistance = in_meters = (1000 * normalized) + (length * 0.5)
		

		distance(finaldistance)
		#print("final: ", finaldistance, " vs. direct: ", in_meters)
	else:
		distance(0)

def getdepth(arraycoordinates,transform_name, transform_len):
	array = arraycoordinates
	newimageRGB = rgb_frame
	if type(array) == int:
		distance(0)
		length(0)
	elif 0>array[0]>IM_WIDTH and 0>array[1]>IM_HEIGTH and array[2]<0 and array[3]<0:
		distance(0)
		length(0)
	elif transform_len>3:
		distance(0)
		length(0)
	elif 0<array[0]<IM_WIDTH and 0<array[1]<IM_HEIGTH and array[2]>0 and array[3]>0:
		#print("I am here")
		ycenter = array[0] + (array[2] * 0.35)
		xcenter = array[1] + (array[3] * 0.5)

		pixel = depthimage[int(xcenter),int(ycenter)]
		normalized = (pixel[2] + pixel[1] * 256 + pixel[0] * 256 * 256) / (256 * 256 * 256 - 1)
		in_meters = 1000 * normalized



		frame_of_interest = newimageRGB[array[1]:(array[1]+array[3]),array[0]:(array[0]+array[2])]
		frame_of_interest_grayscaled = cv2.cvtColor(frame_of_interest, cv2.COLOR_BGR2GRAY)
		gaus_threshold = cv2.adaptiveThreshold(frame_of_interest_grayscaled, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115,1)
		
		#print("array[3]: ",array[3], " and range(array[3]: ", range(array[3]))

		nearest = 1000
		furthest = 0


		for i in range(array[3]):
			for j in range(array[2]):
				if gaus_threshold[i,j] == 0:
					if (array[0]+i)>IM_HEIGTH:
						i = i -1
					if (array[1]+j)>IM_WIDTH:
						j = j - 1
					pixel_length = depthimage[(array[1]+i),(array[0]+j)]
					calc = (pixel_length[2] + pixel_length[1] * 256 + pixel_length[0] * 256 * 256) / (256 * 256 * 256 - 1)
					calc_in_meters = 1000 * calc


					if calc_in_meters < nearest:
						nearest = calc_in_meters
					if calc_in_meters > furthest:
						furthest = calc_in_meters

		#if angle != 0:
			#dist = math.sqrt(in_meters**2 + length**2 +2*in_meters*length*math.cos(angle))
		#else:
			#dist = in_meters
		#print("Distanz: ", in_meters)
		length = furthest-nearest
		if length == -1000:
			distance(in_meters)
		elif transform_name == ['car'] and length > 7:
			dist = in_meters + 2
			distance(dist)
			length(4.0)
		elif transform_name == ['person'] and length > 2:
			dist = in_meters + 0.3
			distance(dist)
			length(0.6)
		else:
			dist = in_meters + length * 0.5
			distance(dist)
			length(length)
		
	else:
		distance(0)
		length(0)

def yolostart(imagevariable):
	global rgb_frame
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
		#print("[INFO] YOLO took {:.6f} seconds".format(end - start))
		# initialize our lists of detected bounding boxes, confidences, and
		# class IDs, respectively
		boxes = []
		confidences = []
		classIDs = [] 

		############
		objIDs = []
		rects = []
		############

		# loop over each of the layer outputs
		for output in layerOutputs:
			# loop over each of the detections
			for detection in output:

				#classNameString = ""
				#classNameString = classNameString.join(np.argmax(scores))
				#if(classNameString == "umbrella"):
				#	print("umbrella !!!!!!!!!!!!!!!!!!")
				# extract the class ID and confidence (i.e., probability) of
				# the current object detection
				scores = detection[5:]
				confidence = 0
				if (np.argmax(scores) == 0 or
					np.argmax(scores) == 1 or
					np.argmax(scores) == 2 or
					np.argmax(scores) == 3 or
					np.argmax(scores) == 7):

					classID = np.argmax(scores)
					confidence = scores[classID]
				# filter out weak predictions by ensuring the detected
				# probability is greater than the minimum probability
				#print(classID)
				if confidence > CONFIDENCE:
					# scale the bounding box coordinates back relative to the
					# size of the image, keeping in mind that YOLO actually
					# returns the center (x, y)-coordinates of the bounding
					# box followed by the boxes' width and height
					box = detection[0:4] * np.array([W, H, W, H])
					(centerX, centerY, width, height) = box.astype("int")
					########
					box2 = [centerX-width/2, centerX+width/2, centerY-height/2, centerY+height/2]
					rects.append(box2)
					#######
					# use the center (x, y)-coordinates to derive the top and
					# and left corner of the bounding box
					x = int(centerX - (width / 2))
					y = int(centerY - (height / 2))
					# update our list of bounding box coordinates, confidences,
					# and class IDs
					boxes.append([x, y, int(width), int(height)])
					confidences.append(float(confidence))
					classIDs.append(classID)
		########
		objects = ct.update(rects)
		iiiDs = []
		for(objIDs, centroid) in objects.items():
			#print(objIDs)
			#print(type(objIDs))
			iiiDs.append(objIDs)
		#print(iiiDs)
		if counter > 0 :            
			# apply non-maxima suppression to suppress weak, overlapping bounding
			# boxes
			idxs = cv2.dnn.NMSBoxes(boxes, confidences,CONFIDENCE,
				THRESHOLD)
			j = []	#changed
			s = []
			k = []
			i_D_Var = []
			objSum(len(idxs))
			# ensure at least one detection exists
			if len(idxs) > 0 & len(idxs) < 100:
				# loop over the indexes we are keeping
				for i in idxs.flatten():
					if i < 20:
						# extract the bounding box coordinates
						(x, y) = (boxes[i][0], boxes[i][1])
						(w, h) = (boxes[i][2], boxes[i][3])
						s.append([LABELS[classIDs[i]]])	#changed
						j.append([boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]])
						k.append([confidences[i]])
						#print("Index: {}".format(i))
						#print(s)
						#print(len(iiiDs))
						if i < len(iiiDs):
							i_D_Var.append([iiiDs[i]])
						yoloVariables(j, k, s, i_D_Var)
						process2(1)
						rgb_frame = imagevariable
			else:
				process1(2)

		count(2)        
	if counter < 1:
		process1(2)
	return None  


def depthAlgo(image2):
	if step2 == 1:
		array = np.array(image2.raw_data)
		i = array.reshape(IM_HEIGTH,IM_WIDTH,4)
		i2 = i[:,:,:3]
		arrayofdepthcamera(i2)
		process2(0)
		process3(1)
	return None

# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
	def __init__(self, carla_world, hud, actor_filter):
		self.world = carla_world
		self.map = self.world.get_map()
		self.hud = hud
		self.player = None
		self.npc1 = None
		self.npc2 = None
		self.player_position = carla.Location()
		self.npc1_position = carla.Location()
		self.npc2_position = carla.Location()
		self.walker1 = None
		self.collision_sensor = None
		self.lane_invasion_sensor = None
		self.gnss_sensor = None
		self.camera_manager = None
		self.rgb_sensor = None
		self.depth_sensor = None
		self._weather_presets = find_weather_presets()
		self._weather_index = 0
		self._actor_filter = actor_filter
		self.restart()
		self.world.on_tick(hud.on_world_tick)
		self.recording_enabled = False
		self.recording_start = 0
		
		

	
	def restart(self):
		# Keep same camera config if the camera manager exists.
		cam_index = self.camera_manager.index if self.camera_manager is not None else 0
		cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
		# Get a blueprint.

		blueprint = self.world.get_blueprint_library().filter(self._actor_filter)
	
		blueprintPlayer = blueprint[8]
		blueprintNpc1 = blueprint[1]
		blueprintNpc2 = blueprint[14]

		blueprintPlayer.set_attribute('role_name', 'hero')
		blueprintNpc1.set_attribute('role_name', 'npc1')
		blueprintNpc2.set_attribute('role_name', 'npc2')

		blueprintNpc1.set_attribute('color', '17,37,103')
		#blueprintNpc2.set_attribute('color', '17,37,103')

		while self.player is None:
			spawn_points = self.map.get_spawn_points() 
			spawn_point = carla.Transform(carla.Location(x=119.600000, y=8.330196, z=1.843102), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
			self.player = self.world.spawn_actor(blueprintPlayer, spawn_point)
			self.vehicle_control = carla.VehicleControl()
			self.vehicle_control.throttle = 1.0
			

	   
		# Spawn the npc cars
			spawn_point = carla.Transform(carla.Location(x=219.600000, y=12.800000, z=1.843102), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
			self.npc1 = self.world.try_spawn_actor(blueprintNpc1, spawn_point)
			spawn_point = carla.Transform(carla.Location(x=214.123000, y=12.883151, z=1.843102), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
			self.npc2 = self.world.spawn_actor(blueprintNpc2, spawn_point)
	   
		# Spawn the walker
			walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
			blueprintsWalkers = self.world.get_blueprint_library().filter("walker.*")
			blueprintsWalkers = blueprintsWalkers[0]
			if blueprintsWalkers.has_attribute('is_invincible'):
				blueprintsWalkers.set_attribute('is_invincible', 'false')
			spawn_point = carla.Transform(carla.Location(x=223.013613, y=15.755237, z=2.063102), carla.Rotation(pitch=0.000000, yaw=270.855823, roll=0.000000))
			self.walker1 = self.world.spawn_actor(blueprintsWalkers, spawn_point)

			self.world.wait_for_tick()
			

		
		# Set up walker control by direct input 
			self.control = carla.WalkerControl()
			self.control.speed = 1.39
			self.control.direction = carla.Vector3D(x=0.000000, y=-1.000000, z=0.000000)
		
			
		# Set up the sensors.
		self.collision_sensor = CollisionSensor(self.player, self.hud)
		self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
		self.gnss_sensor = GnssSensor(self.player)
		self.camera_manager = CameraManager(self.player, self.hud)
		self.camera_manager.transform_index = cam_pos_index
		self.camera_manager.set_sensor(cam_index, notify=False)
		actor_type = get_actor_display_name(self.player)
		self.hud.notification(actor_type)
		self.rgb_sensor = RgbSensor(self.player)
		self.depth_sensor = DepthSensor(self.player)


	def next_weather(self, reverse=False):
		self._weather_index += -1 if reverse else 1
		self._weather_index %= len(self._weather_presets)
		preset = self._weather_presets[self._weather_index]
		self.hud.notification('Weather: %s' % preset[1])
		self.player.get_world().set_weather(preset[0])

	def tick(self, clock):
		self.hud.tick(self, clock)

	def render(self, display):
		self.camera_manager.render(display)
		self.hud.render(display)

	def destroy_sensors(self):
		self.camera_manager.sensor.destroy()
		self.camera_manager.sensor = None
		self.camera_manager.index = None
	
		
   
	def destroy(self):
		actors = [
			self.camera_manager.sensor,
			self.collision_sensor.sensor,
			self.lane_invasion_sensor.sensor,
			self.gnss_sensor.sensor,
			self.rgb_sensor.sensor,
			self.depth_sensor.sensor,
			self.player,
			self.npc1,
			self.npc2,
			self.walker1]
		for actor in actors:
			if actor is not None:
				actor.destroy()



class Feature(object):
	def __init__(self,world,actor):
		fov_angle = 30

		player_transform = world.player.get_transform()
		player_yaw = player_transform.rotation.yaw
		player_location = world.player.get_location()  
		player_location.x = math.cos(player_yaw * math.pi/180) * world.player.bounding_box.extent.x + player_location.x
		player_location.y = math.sin(player_yaw * math.pi/180) * world.player.bounding_box.extent.x + player_location.y

		actor_location = actor.get_location()	
		actor_transform = actor.get_transform()
		actor_yaw = actor_transform.rotation.yaw
		Xcoord = carla.Location()
		Xcoord.x = actor.bounding_box.extent.x * math.cos(actor_yaw * math.pi/180)
		Xcoord.y = actor.bounding_box.extent.x * math.sin(actor_yaw * math.pi/180)
		
		Ycoord = carla.Location()
		Ycoord.y = actor.bounding_box.extent.y * math.cos(actor_yaw * math.pi/180)
		Ycoord.x = actor.bounding_box.extent.y * math.sin(actor_yaw * math.pi/180)

		
		FR = carla.Location()
		FR = (Xcoord + Ycoord + actor_location) - player_location
		angle = math.atan2(FR.y, FR.x) * 180 / math.pi
		if FR.x<=200 and abs(angle) < fov_angle:
			FRbool = 1
		else:
			FRbool = 0
	
		FM = carla.Location()
		FM = (Xcoord + actor_location) - player_location
		angle = math.atan2(FM.y, FM.x) * 180 / math.pi
		if FM.x<=200 and abs(angle) < fov_angle:
			FMbool = 1
		else:
			FMbool = 0
	
		FL = carla.Location()
		FL = (Xcoord - Ycoord + actor_location) - player_location
		angle = math.atan2(FL.y, FL.x) * 180 / math.pi
		if FL.x<=200 and abs(angle) < fov_angle:
			FLbool = 1
		else:
			FLbool = 0


		
		MR = carla.Location()
		MR = (Ycoord + actor_location) - player_location
		angle = math.atan2(MR.y, MR.x) * 180 / math.pi
		if MR.x<=200 and abs(angle) < fov_angle:
			MRbool = 1
		else:
			MRbool = 0
	

	
		ML = carla.Location()
		ML = ( -1*Ycoord + actor_location) - player_location	
		angle = math.atan2(ML.y, ML.x) * 180 / math.pi
		if ML.x<=200 and abs(angle) < fov_angle:
			MLbool = 1
		else:
			MLbool = 0
	
		
		RR = carla.Location()
		RR = (-1*Xcoord + Ycoord + actor_location) - player_location	
		angle = math.atan2(RR.y, RR.x) * 180 / math.pi
		if RR.x<=200 and abs(angle) < fov_angle:
			RRbool = 1
		else:
			RRbool = 0
	
		RM = carla.Location()
		RM = (-1*Xcoord + actor_location) - player_location	
		angle = math.atan2(RM.y, RM.x) * 180 / math.pi
		if RM.x<=200 and abs(angle) < fov_angle:
			RMbool = 1
		else:
			RMbool = 0
	
		RL = carla.Location()
		RL = (-1*Xcoord - Ycoord + actor_location) - player_location	
		angle = math.atan2(RL.y, RL.x) * 180 / math.pi
		if RL.x<=200 and abs(angle) < fov_angle:
			RLbool = 1
		else:
			RLbool = 0



	
		self.FR = FRbool
		self.FM = FMbool
		self.FL = FLbool
		self.MR = MRbool
		self.ML = MLbool
		self.RR = RRbool
		self.RM = RMbool
		self.RL = RLbool
		self.FRval = FR
		self.FMval = FM
		self.FLval = FL
		self.MRval = MR
		self.MLval = ML
		self.RRval = RR
		self.RMval = RM
		self.RLval = RL
		
	
	def check_exist(self):
		val = self.FR + self.FM + self.FM + self.MR + self.ML + self.RR + self.RM + self.RL
		if val > 0:
			return 1

class Feature2:
	def __init__(self,world,a1,a2,a3):   #currently this only works when a1=Feature npc1 a2=Feature npc2 a3=Feature walker1
		self.values = [a1.FRval,a1.FMval,a1.FLval,a1.MRval,a1.MLval,a1.RRval,a1.RMval,a1.RLval,
			a2.FRval,a2.FMval,a2.FLval,a2.MRval,a2.MLval,a2.RRval,a2.RMval,a2.RLval,
			a3.FRval,a3.FMval,a3.FLval,a3.MRval,a3.MLval,a3.RRval,a3.RMval,a3.RLval]
		self.bools = [a1.FR,a1.FM,a1.FL,a1.MR,a1.ML,a1.RR,a1.RM,a1.RL,
			a2.FR,a2.FM,a2.FL,a2.MR,a2.ML,a2.RR,a2.RM,a2.RL,
			a3.FR,a3.FM,a3.FL,a3.MR,a3.ML,a3.RR,a3.RM,a3.RL]

		idx = 0
		while(idx <= 16):
			values_dist=[]
			for v in self.values[0:8]:
				v = math.sqrt(v.x ** 2 + v.y ** 2)
				values_dist.append(v)
				
			valindex = numpy.argsort(values_dist)
			
			if valindex[0] == 0:
				self.corr_val(idx,1,a1,a2,a3)
				self.corr_val(idx+1,1,a1,a2,a3)
				self.corr_val(idx+2,1,a1,a2,a3)
				self.corr_val(idx+3,1,a1,a2,a3)
				self.corr_val(idx+4,0,a1,a2,a3)
				self.corr_val(idx+5,1,a1,a2,a3)
				self.corr_val(idx+6,0,a1,a2,a3)
				self.corr_val(idx+7,0,a1,a2,a3)
			elif valindex[0] == 1:
				self.corr_val(idx,1,a1,a2,a3)
				self.corr_val(idx+1,1,a1,a2,a3)
				self.corr_val(idx+2,1,a1,a2,a3)
				self.corr_val(idx+3,0,a1,a2,a3)
				self.corr_val(idx+4,0,a1,a2,a3)
				self.corr_val(idx+5,0,a1,a2,a3)
				self.corr_val(idx+6,0,a1,a2,a3)
				self.corr_val(idx+7,0,a1,a2,a3)
			elif valindex[0] == 2:
				self.corr_val(idx,1,a1,a2,a3)
				self.corr_val(idx+1,1,a1,a2,a3)
				self.corr_val(idx+2,1,a1,a2,a3)
				self.corr_val(idx+3,0,a1,a2,a3)
				self.corr_val(idx+4,1,a1,a2,a3)
				self.corr_val(idx+5,0,a1,a2,a3)
				self.corr_val(idx+6,0,a1,a2,a3)
				self.corr_val(idx+7,1,a1,a2,a3)
			elif valindex[0] == 3:
				self.corr_val(idx,1,a1,a2,a3)
				self.corr_val(idx+1,0,a1,a2,a3)
				self.corr_val(idx+2,0,a1,a2,a3)
				self.corr_val(idx+3,1,a1,a2,a3)
				self.corr_val(idx+4,0,a1,a2,a3)
				self.corr_val(idx+5,0,a1,a2,a3)
				self.corr_val(idx+6,0,a1,a2,a3)
				self.corr_val(idx+7,1,a1,a2,a3)
			elif valindex[0] == 4:
				self.corr_val(idx,0,a1,a2,a3)
				self.corr_val(idx+1,0,a1,a2,a3)
				self.corr_val(idx+2,1,a1,a2,a3)
				self.corr_val(idx+3,0,a1,a2,a3)
				self.corr_val(idx+4,1,a1,a2,a3)
				self.corr_val(idx+5,0,a1,a2,a3)
				self.corr_val(idx+6,0,a1,a2,a3)
				self.corr_val(idx+7,1,a1,a2,a3)
			elif valindex[0] == 5:
				self.corr_val(idx,1,a1,a2,a3)
				self.corr_val(idx+1,0,a1,a2,a3)
				self.corr_val(idx+2,0,a1,a2,a3)
				self.corr_val(idx+3,1,a1,a2,a3)
				self.corr_val(idx+4,0,a1,a2,a3)
				self.corr_val(idx+5,1,a1,a2,a3)
				self.corr_val(idx+6,1,a1,a2,a3)
				self.corr_val(idx+7,1,a1,a2,a3)
			elif valindex[0] == 6:
				self.corr_val(idx,0,a1,a2,a3)
				self.corr_val(idx+1,0,a1,a2,a3)
				self.corr_val(idx+2,0,a1,a2,a3)
				self.corr_val(idx+3,0,a1,a2,a3)
				self.corr_val(idx+4,0,a1,a2,a3)
				self.corr_val(idx+5,1,a1,a2,a3)
				self.corr_val(idx+6,1,a1,a2,a3)
				self.corr_val(idx+7,1,a1,a2,a3)
			elif valindex[0] == 7:
				self.corr_val(idx,0,a1,a2,a3)
				self.corr_val(idx+1,0,a1,a2,a3)
				self.corr_val(idx+2,1,a1,a2,a3)
				self.corr_val(idx+3,0,a1,a2,a3)
				self.corr_val(idx+4,1,a1,a2,a3)
				self.corr_val(idx+5,1,a1,a2,a3)
				self.corr_val(idx+6,1,a1,a2,a3)
				self.corr_val(idx+7,1,a1,a2,a3)
		
			idx = idx + 8
			
		
		self.bools = [a1.FR,a1.FM,a1.FL,a1.MR,a1.ML,a1.RR,a1.RM,a1.RL,
			a2.FR,a2.FM,a2.FL,a2.MR,a2.ML,a2.RR,a2.RM,a2.RL,
			a3.FR,a3.FM,a3.FL,a3.MR,a3.ML,a3.RR,a3.RM,a3.RL]






#Walker 1 Hardcode
		von = 16
		bis = 24
		count = von
		for v in self.values[von:bis]:
			if (math.atan2(v.y, v.x) > math.atan2(self.values[13].y, self.values[13].x) or math.atan2(v.y, v.x) < math.atan2(self.values[2].y, self.values[2].x)):
				self.corr_val(count,1,a1,a2,a3)
			else:
				self.corr_val(count,0,a1,a2,a3)
			count = count+1
			
			

		
		
		
	def corr_val(self,valindex,visible,a1,a2,a3):

		if visible == 1 and self.bools[valindex] == 1 :
			visible = 1
		else:
			visible = 0

	
		actor = a1
		if valindex > 7 and valindex <= 15:
			valindex = valindex - 8
			actor = a2
		elif valindex > 15:
			valindex = valindex - 16
			actor = a3

		if valindex == 0 and visible == 0:
			actor.FR = 0
		elif valindex == 0 and visible == 1:
			actor.FR = 1
		elif valindex == 1 and visible == 0:
			actor.FM = 0
		elif valindex == 1 and visible == 1:
			actor.FM = 1
		elif valindex == 2 and visible == 0:
			actor.FL = 0
		elif valindex == 2 and visible == 1:
			actor.FL = 1
		elif valindex == 3 and visible == 0:
			actor.MR = 0
		elif valindex == 3 and visible == 1:
			actor.MR = 1
		elif valindex == 4 and visible == 0:
			actor.ML = 0
		elif valindex == 4 and visible == 1:
			actor.ML = 1
		elif valindex == 5 and visible == 0:
			actor.RR = 0
		elif valindex == 5 and visible == 1:
			actor.RR = 1
		elif valindex == 6 and visible == 0:
			actor.RM = 0
		elif valindex == 6 and visible == 1:
			actor.RM = 1
		elif valindex == 7 and visible == 0:
			actor.RL = 0
		elif valindex == 7 and visible == 1:
			actor.RL = 1
		

class Dimension:
	def __init__(self,actor):
		self.length = actor.bounding_box.extent.x *2
		self.height = actor.bounding_box.extent.z *2
		self.width = actor.bounding_box.extent.y *2

class Geometric:
	def __init__(self,world,actor):
		
		
		if actor == world.player:
			tempxy = actor.get_location() 
			self.x = tempxy.x
			self.y = tempxy.y
			tempv = actor.get_velocity()
			self.vx = tempv.x
			self.vy = tempv.y
			tempa = actor.get_acceleration()
			self.ax = tempa.x
			self.ay = tempa.y
			actort = actor.get_transform()  
			self.yaw = actort.rotation.yaw 
			tempav = actor.get_angular_velocity()
			self.yawrate = tempav
		else:
			tempxy = actor.get_location() - world.player.get_location()
			self.x = tempxy.x
			self.y = tempxy.y
			tempv = actor.get_velocity() - world.player.get_velocity()
			self.vx = tempv.x
			self.vy = tempv.y
			tempa = actor.get_acceleration() - world.player.get_acceleration()
			self.ax = tempa.x
			self.ay = tempa.y
			playert = world.player.get_transform()
			actort = actor.get_transform()  
			self.yaw = actort.rotation.yaw - playert.rotation.yaw
			tempav = actor.get_angular_velocity() - world.player.get_angular_velocity()
			self.yawrate = tempav


def Object_List_Talker(world,args):

	pub = rospy.Publisher('simulation', ObjectsList, queue_size=100) #
	rospy.init_node('camera',anonymous=False)  # Initiate the node camera and anonymous true permitt openinig this node a lot of time including number in the end of the node name  
   #rate=rospy.Rate(50)  #50 hz

   #while not rospy.is_shutdown():

	b=ObjectsList() # Erstellen des Objekts ObjectsList (enthaelt alle Objekte)
	a1=ObjectList() # Erstellen eines Objektes von der Klasse ObjectList
	a1.obj_id= 2	# Zuweisen der ID

	 
	Npc1_geo = Geometric(world,world.npc1)
	Npc1_dim = Dimension(world.npc1)
	Npc1_fea = Feature(world,world.npc1)
	Npc2_geo = Geometric(world,world.npc2)
	Npc2_dim = Dimension(world.npc2)
	Npc2_fea = Feature(world,world.npc2)
	Walker1_geo = Geometric(world,world.walker1)
	Walker1_dim = Dimension(world.walker1)
	Walker1_fea = Feature(world,world.walker1)
	Player_geo = Geometric(world,world.player)
	FE2 = Feature2(world,Npc1_fea,Npc2_fea,Walker1_fea)
	

	a1.geometric.x = Npc1_geo.x
	a1.geometric.y = Npc1_geo.y
	a1.geometric.vx = Npc1_geo.vx
	a1.geometric.vy = Npc1_geo.vy
	a1.geometric.ax = Npc1_geo.ax
	a1.geometric.ay = Npc1_geo.ay
	a1.geometric.yaw= Npc1_geo.yaw
	#a1.geometric.yawrate = Npc1_geo.yawrate
	a1.dimension.length = Npc1_dim.length
	a1.dimension.height = Npc1_dim.height
	a1.dimension.width = Npc1_dim.width
	a1.features.FR = Npc1_fea.FR
	a1.features.FM = Npc1_fea.FM
	a1.features.FL = Npc1_fea.FL
	a1.features.MR = Npc1_fea.MR
	a1.features.ML = Npc1_fea.ML
	a1.features.RR = Npc1_fea.RR
	a1.features.RM = Npc1_fea.RM
	a1.features.RL = Npc1_fea.RL
	a1.classification.car = 1   
	a1.prop_existence = 1
	a1.prop_mov = 0


	
	a2=ObjectList()
	a2.obj_id= 1


	a2.geometric.x = Npc2_geo.x
	a2.geometric.y = Npc2_geo.y
	a2.geometric.vx = Npc2_geo.vx
	a2.geometric.vy = Npc2_geo.vy
	a2.geometric.ax = Npc2_geo.ax
	a2.geometric.ay = Npc2_geo.ay
	a2.geometric.yaw= Npc2_geo.yaw
	#a2.geometric.yawrate = Npc2_geo.yawrate
	a2.dimension.length = Npc2_dim.length
	a2.dimension.height = Npc2_dim.height
	a2.dimension.width = Npc2_dim.width
	a2.features.FR = Npc2_fea.FR
	a2.features.FM = Npc2_fea.FM
	a2.features.FL = Npc2_fea.FL
	a2.features.MR = Npc2_fea.MR
	a2.features.ML = Npc2_fea.ML
	a2.features.RR = Npc2_fea.RR
	a2.features.RM = Npc2_fea.RM
	a2.features.RL = Npc2_fea.RL
	a2.classification.car = 1
	a2.prop_existence = 1
	a2.prop_mov = 0


	a3=ObjectList()
	a3.obj_id= 3


	a3.geometric.x = Walker1_geo.x
	a3.geometric.y = Walker1_geo.y
	a3.geometric.vx = Walker1_geo.vx
	a3.geometric.vy = Walker1_geo.vy
	a3.geometric.ax = Walker1_geo.ax
	a3.geometric.ay = Walker1_geo.ay
	a3.geometric.yaw= Walker1_geo.yaw
	#a3.geometric.yawrate = Walker1_geo.yawrate
	a3.dimension.length = Walker1_dim.length
	a3.dimension.height = Walker1_dim.height
	a3.dimension.width = Walker1_dim.width
	a3.features.FR = Walker1_fea.FR
	a3.features.FM = Walker1_fea.FM
	a3.features.FL = Walker1_fea.FL
	a3.features.MR = Walker1_fea.MR
	a3.features.ML = Walker1_fea.ML
	a3.features.RR = Walker1_fea.RR
	a3.features.RM = Walker1_fea.RM
	a3.features.RL = Walker1_fea.RL
	a3.classification.pedestrian = 1
	a3.prop_existence = 1
	a3.prop_mov = 1


	b.header.frame_id = "ObjectList_GroundTruth"


# hier werden die Objekte der Objektliste hinzugefuegt mittels append
	if Npc2_fea.check_exist() == 1 or args.visible:
		b.obj_list.append(a2)
	if Npc1_fea.check_exist() == 1 or args.visible:
		b.obj_list.append(a1)
	if Walker1_fea.check_exist() == 1 or args.visible:
		b.obj_list.append(a3)
	b.header.stamp = rospy.Time.now()

# hier wird die Nachricht gepublished und ein Node kann diese Nachricht abonnieren
	if Npc1_fea.check_exist() == 1 or Npc2_fea.check_exist() == 1 or Walker1_fea.check_exist() == 1:
		pub.publish(b)
	rospy.loginfo(b)
	

	pub = rospy.Publisher('egovehicle', ObjectsList, queue_size=100) #
	rospy.init_node('camera',anonymous=False)  
 

	b=ObjectsList() # Erstellen des Objekts ObjectsList (enthaelt alle Obj)

# ab hier werden die Eigenschaften eines Objektes zugewiesen
	 

	a0=geo()

	a0.x = Player_geo.x
	a0.y = Player_geo.y
	a0.vx = Player_geo.vx
	a0.vy = Player_geo.vx
	a0.ax = Player_geo.ax
	a0.ay = Player_geo.ay

	

# der ObjektListe werden noch charakteristische Eigenschaften zugewiesen wie			# TimeStamp und Frame ID

	b.header.stamp = rospy.Time.now()
	b.header.frame_id = "ObjectListego_GroundTruth"

# hier werden die Objekte der Objektliste hinzugefuegt mittels append
	b.ego_geometric.append(a0)
	b.header.stamp = rospy.Time.now()
# hier wird die Nachricht gepublished und ein Node kann diese Nachricht abonnieren
	pub.publish(b)
	#rospy.loginfo(b)
	
# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
	def __init__(self, world):
		world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

	def parse_events(self):
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				return True
			elif event.type == pygame.KEYUP:
				if self._is_quit_shortcut(event.key):
					return True

	@staticmethod
	def _is_quit_shortcut(key):
		return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
	def __init__(self, width, height):
		self.dim = (width, height)
		font = pygame.font.Font(pygame.font.get_default_font(), 20)
		font_name = 'courier' if os.name == 'nt' else 'mono'
		fonts = [x for x in pygame.font.get_fonts() if font_name in x]
		default_font = 'ubuntumono'
		mono = default_font if default_font in fonts else fonts[0]
		mono = pygame.font.match_font(mono)
		self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
		self._notifications = FadingText(font, (width, 40), (0, height - 40))
		self.help = HelpText(pygame.font.Font(mono, 24), width, height)
		self.server_fps = 0
		self.frame = 0
		self.simulation_time = 0
		self._show_info = True
		self._info_text = []
		self._server_clock = pygame.time.Clock()

	def on_world_tick(self, timestamp):
		self._server_clock.tick()
		self.server_fps = self._server_clock.get_fps()
		self.frame = timestamp.frame
		self.simulation_time = timestamp.elapsed_seconds

	def tick(self, world, clock):
		self._notifications.tick(world, clock)
		if not self._show_info:
			return
		t = world.player.get_transform()
		v = world.player.get_velocity()
		c = world.player.get_control()
		heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
		heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
		heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
		heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
		colhist = world.collision_sensor.get_collision_history()
		collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
		max_col = max(1.0, max(collision))
		collision = [x / max_col for x in collision]
		vehicles = world.world.get_actors().filter('vehicle.*')
		self._info_text = [
			'Server:  % 16.0f FPS' % self.server_fps,
			'Client:  % 16.0f FPS' % clock.get_fps(),
			'',
			'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
			'Map:	 % 20s' % world.map.name,
			'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
			'',
			'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)),
			u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
			'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
			'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
			'Height:  % 18.0f m' % t.location.z,
			'']
		if isinstance(c, carla.VehicleControl):
			self._info_text += [
				('Throttle:', c.throttle, 0.0, 1.0),
				('Steer:', c.steer, -1.0, 1.0),
				('Brake:', c.brake, 0.0, 1.0),
				('Reverse:', c.reverse),
				('Hand brake:', c.hand_brake),
				('Manual:', c.manual_gear_shift),
				'Gear:		%s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
		elif isinstance(c, carla.WalkerControl):
			self._info_text += [
				('Speed:', c.speed, 0.0, 5.556),
				('Jump:', c.jump)]
		self._info_text += [
			'',
			'Collision:',
			collision,
			'',
			'Number of vehicles: % 8d' % len(vehicles)]
		if len(vehicles) > 1:
			self._info_text += ['Nearby vehicles:']

			def distance(l): return math.sqrt(
				(l.x - t.location.x) ** 2 + (l.y - t.location.y) ** 2 + (l.z - t.location.z) ** 2)
			vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
			for d, vehicle in sorted(vehicles):
				if d > 200.0:
					break
				vehicle_type = get_actor_display_name(vehicle, truncate=22)
				self._info_text.append('% 4dm %s' % (d, vehicle_type))

	def toggle_info(self):
		self._show_info = not self._show_info

	def notification(self, text, seconds=2.0):
		self._notifications.set_text(text, seconds=seconds)

	def error(self, text):
		self._notifications.set_text('Error: %s' % text, (255, 0, 0))

	def render(self, display):
		if self._show_info:
			info_surface = pygame.Surface((220, self.dim[1]))
			info_surface.set_alpha(100)
			display.blit(info_surface, (0, 0))
			v_offset = 4
			bar_h_offset = 100
			bar_width = 106
			for item in self._info_text:
				if v_offset + 18 > self.dim[1]:
					break
				if isinstance(item, list):
					if len(item) > 1:
						points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
						pygame.draw.lines(display, (255, 136, 0), False, points, 2)
					item = None
					v_offset += 18
				elif isinstance(item, tuple):
					if isinstance(item[1], bool):
						rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
						pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
					else:
						rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
						pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
						f = (item[1] - item[2]) / (item[3] - item[2])
						if item[2] < 0.0:
							rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
						else:
							rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
						pygame.draw.rect(display, (255, 255, 255), rect)
					item = item[0]
				if item:  # At this point has to be a str.
					surface = self._font_mono.render(item, True, (255, 255, 255))
					display.blit(surface, (8, v_offset))
				v_offset += 18
		self._notifications.render(display)
		self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
	def __init__(self, font, dim, pos):
		self.font = font
		self.dim = dim
		self.pos = pos
		self.seconds_left = 0
		self.surface = pygame.Surface(self.dim)

	def set_text(self, text, color=(255, 255, 255), seconds=2.0):
		text_texture = self.font.render(text, True, color)
		self.surface = pygame.Surface(self.dim)
		self.seconds_left = seconds
		self.surface.fill((0, 0, 0, 0))
		self.surface.blit(text_texture, (10, 11))

	def tick(self, _, clock):
		delta_seconds = 1e-3 * clock.get_time()
		self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
		self.surface.set_alpha(500.0 * self.seconds_left)

	def render(self, display):
		display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
	def __init__(self, font, width, height):
		lines = __doc__.split('\n')
		self.font = font
		self.dim = (680, len(lines) * 22 + 12)
		self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
		self.seconds_left = 0
		self.surface = pygame.Surface(self.dim)
		self.surface.fill((0, 0, 0, 0))
		for n, line in enumerate(lines):
			text_texture = self.font.render(line, True, (255, 255, 255))
			self.surface.blit(text_texture, (22, n * 22))
			self._render = False
		self.surface.set_alpha(220)

	def toggle(self):
		self._render = not self._render

	def render(self, display):
		if self._render:
			display.blit(self.surface, self.pos)

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
	def __init__(self, parent_actor, hud):
		self.sensor = None
		self.history = []
		self._parent = parent_actor
		self.hud = hud
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.collision')
		self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

	def get_collision_history(self):
		history = collections.defaultdict(int)
		for frame, intensity in self.history:
			history[frame] += intensity
		return history

	@staticmethod
	def _on_collision(weak_self, event):
		self = weak_self()
		if not self:
			return
		actor_type = get_actor_display_name(event.other_actor)
		self.hud.notification('Collision with %r' % actor_type)
		impulse = event.normal_impulse
		intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
		self.history.append((event.frame, intensity))
		if len(self.history) > 4000:
			self.history.pop(0)

# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
	def __init__(self, parent_actor, hud):
		self.sensor = None
		self._parent = parent_actor
		self.hud = hud
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
		self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

	@staticmethod
	def _on_invasion(weak_self, event):
		self = weak_self()
		if not self:
			return
		lane_types = set(x.type for x in event.crossed_lane_markings)
		text = ['%r' % str(x).split()[-1] for x in lane_types]
		self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self._parent = parent_actor
		self.lat = 0.0
		self.lon = 0.0
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.gnss')
		self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
										attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

	@staticmethod
	def _on_gnss_event(weak_self, event):
		self = weak_self()
		if not self:
			return
		self.lat = event.latitude
		self.lon = event.longitude

# ==============================================================================
# -- RGB-Camera TP2 -------------------------------------------------------------
# ==============================================================================

class RgbSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self._parent = parent_actor
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.camera.rgb')
		bp.set_attribute('image_size_x', '800')
		bp.set_attribute('image_size_y', '600')
		bp.set_attribute('sensor_tick', '0.5')
		self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
										attach_to=self._parent)
		self.sensor.listen(lambda image: (myprocess(image)))		

def myprocess(image):
	#Configurate the RGB-Image and starting the yolo-function
	array = np.array(image.raw_data)
	i2 = array.reshape(IM_HEIGTH,IM_WIDTH,4)
	frame = i2[:,:,:3]
	yolostart(frame)
	return frame 

class DepthSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self._parent = parent_actor
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.camera.depth')
		bp.set_attribute('image_size_x', '800')
		bp.set_attribute('image_size_y', '600')
		bp.set_attribute('sensor_tick', '0.5')
		self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
										attach_to=self._parent)
		self.sensor.listen(lambda image2: (myprocess2(image2)))	

def myprocess2(image2):
	#Giving deept-Image to depth-function
	depthAlgo(image2)
	return None

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
	def __init__(self, parent_actor, hud):
		self.sensor = None
		self.surface = None
		self._parent = parent_actor
		self.hud = hud
		self.recording = False
		self._camera_transforms = [
			carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
			carla.Transform(carla.Location(x=1.6, z=1.7))]
		self.transform_index = 1
		self.sensors = [
			['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
			['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
			['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
			['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
			['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
			['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
			 'Camera Semantic Segmentation (CityScapes Palette)'],
			['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
		world = self._parent.get_world()
		bp_library = world.get_blueprint_library()
		for item in self.sensors:
			bp = bp_library.find(item[0])
			if item[0].startswith('sensor.camera'):
				bp.set_attribute('image_size_x', str(hud.dim[0]))
				bp.set_attribute('image_size_y', str(hud.dim[1]))
			elif item[0].startswith('sensor.lidar'):
				bp.set_attribute('range', '50')
			item.append(bp)
		self.index = None

	def toggle_camera(self):
		self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
		self.sensor.set_transform(self._camera_transforms[self.transform_index])

	def set_sensor(self, index, notify=True):
		index = index % len(self.sensors)
		needs_respawn = True if self.index is None \
			else self.sensors[index][0] != self.sensors[self.index][0]
		if needs_respawn:
			if self.sensor is not None:
				self.sensor.destroy()
				self.surface = None
			self.sensor = self._parent.get_world().spawn_actor(
				self.sensors[index][-1],
				self._camera_transforms[self.transform_index],
				attach_to=self._parent)
			# We need to pass the lambda a weak reference to self to avoid
			# circular reference.
			weak_self = weakref.ref(self)
			self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
		if notify:
			self.hud.notification(self.sensors[index][2])
		self.index = index

	def next_sensor(self):
		self.set_sensor(self.index + 1)

	def toggle_recording(self):
		self.recording = not self.recording
		self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

	def render(self, display):
		if self.surface is not None:
			display.blit(self.surface, (0, 0))

	@staticmethod
	def _parse_image(weak_self, image):
		self = weak_self()
		if not self:
			return
		if self.sensors[self.index][0].startswith('sensor.lidar'):
			points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
			points = np.reshape(points, (int(points.shape[0] / 3), 3))
			lidar_data = np.array(points[:, :2])
			lidar_data *= min(self.hud.dim) / 100.0
			lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
			lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
			lidar_data = lidar_data.astype(np.int32)
			lidar_data = np.reshape(lidar_data, (-1, 2))
			lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
			lidar_img = np.zeros(lidar_img_size)
			lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
			self.surface = pygame.surfarray.make_surface(lidar_img)
		else:
			image.convert(self.sensors[self.index][1])
			array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
			array = np.reshape(array, (image.height, image.width, 4))
			array = array[:, :, :3]
			array = array[:, :, ::-1]
			self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
		if self.recording:
			image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------
# ==============================================================================

def game_loop(args):
	pygame.init()
	pygame.font.init()
	world = None
	count(0)
	process1(2)
	process2(0)
	process3(0)

	try:
		client = carla.Client(args.host, args.port)
		client.set_timeout(4.0)

		display = pygame.display.set_mode(
			(args.width, args.height),
			pygame.HWSURFACE | pygame.DOUBLEBUF)

		hud = HUD(args.width, args.height)
		world = World(client.get_world(), hud, args.filter)
		controller = KeyboardControl(world)

		#================= Domi instanzen bilden   

		ListeObj2 = []
		startTime = 0
		endTime = 0
		DiffTime = 0
		winkelArray = []
		allClassName = []
		allDistances = []
		allCoordinates = []
		#================= Domi instanzen bilden


		clock = pygame.time.Clock()
		max_speed = 0;
		while True:
			if controller.parse_events():
				return

			# as soon as the server is ready continue!
			world.world.wait_for_tick(10.0)

			world.tick(clock)
			world.render(display)
			pygame.display.flip()
			
			player_pos = world.player.get_location()  

			
			player_velocity = world.player.get_velocity()
			player_velocity = math.sqrt(player_velocity.x ** 2 + player_velocity.y ** 2)
		
			
			
			if player_pos.x < 210:                        #needs rework when object detection works
				if   max_speed == 1:
					world.vehicle_control.manual_gear_shift = True
					world.vehicle_control.gear = 3
					world.vehicle_control.throttle = 0.46
					world.player.apply_control(world.vehicle_control)
				elif max_speed == 0:
					world.vehicle_control.throttle = 1.0
					world.player.apply_control(world.vehicle_control)
					if player_velocity > 34/3.6:
						max_speed = 1
			else: 
				world.vehicle_control.manual_gear_shift = False
				world.vehicle_control.throttle = 0.0
				world.vehicle_control.brake = 1
				world.player.apply_control(world.vehicle_control)



			if player_pos.x > 170.56:
				world.walker1.apply_control(world.control)
	
			if step3 == 1:
				#hier variable

				if counter > 1:

					newObjNumber = objNumber
					newObjCoordinates = objCoordinates
					newObjDetectionName = objDetectionName
					newObjID_Var = objID_Var
					newObjConfidence = objConfidence
					print(newObjDetectionName)

					endTime = time.time()
					DiffTime = endTime - startTime
					print("Zeit für einen Loop");
					print("took {:.6f} seconds".format(DiffTime))
					startTime = time.time()
					setTime(DiffTime)




					for i in range(len(newObjDetectionName)):
						if checkDetectObj(newObjDetectionName[i]) == 1:
							if (len(newObjDetectionName)>3) or (newObjCoordinates[i][0] or newObjCoordinates[i][1])>800 or newObjCoordinates[i] == int:
								distance(0)
								length_def(0)
							else:
								getlength(newObjCoordinates[i], newObjDetectionName[i], len(newObjDetectionName))
								getdistance(newObjCoordinates[i], len(newObjDetectionName), winkelArray)
								#getdepth(newObjCoordinates[i], newObjDetectionName[i], len(newObjDetectionName))
							

							idFound = False
							
							for j in range(len(ListeObj2)):

								if ("".join(ListeObj2[j].iD_Classname) == "".join(newObjDetectionName[i]) and int("".join(map(str,newObjID_Var[i]))) == ListeObj2[j].id_Klasse):
									ListeObj2[j].setAllValues(objDistance,newObjCoordinates[i],newObjDetectionName[i],newObjID_Var[i],float("".join(map(str,newObjConfidence[i]))),objLength)
									idFound = True
									#print(type(float("".join(map(str,newObjConfidence[i])))))

							if idFound == False:
								ListeObj2.append(AllMSG())
								ListeObj2[i].setAllValues(objDistance,newObjCoordinates[i],newObjDetectionName[i],newObjID_Var[i],float("".join(map(str,newObjConfidence[i]))),objLength)
								
						if i == len(newObjDetectionName)-1:
							process3(0)
							process1(2)

					z = 0
					while z < len(ListeObj2):
						#print("Länge der Liste")
						#print(len(ListeObj2))
						if z< len(ListeObj2):
							if ListeObj2[z].geo.x == 0 and ListeObj2[z].geo.y == 0 or not (ListeObj2[z].id_Klasse or ListeObj2[z].id_Klasse ==0):
								del ListeObj2[z]
								#del winkelArray[z]
								continue
							if ListeObj2[z].stillChanging != True:
								#ListeObj2.pop(z)
								del ListeObj2[z]
								#del winkelArray[z]
						z +=1
######################################################################
					del winkelArray[:]
					del allClassName[:]
					del allDistances[:]
					del allCoordinates[:]
					z = 0
					for z in range(len(ListeObj2)):
						winkelArray.append(ListeObj2[z].geo.yaw)
						allClassName.append(ListeObj2[z].iD_Classname)
						allDistances.append(ListeObj2[z].dist_)
						allCoordinates.append(ListeObj2[z].coordinates_)
					###################################
					#Mögliche Methode für Länge
					###################################
					
######################################################################
					z = 0
					print("########################################")
					for z in range(len(ListeObj2)):
						ListeObj2[z].changeTrigger()

					#print("Alle Werte:")
					#print(winkelArray)
					#print("Domi's Info: ",allClassName, " Denis's Info: ", newObjDetectionName)
					#print(allDistances)
					#print("Domi's Info: ", allCoordinates, " Denis's Info: ", newObjCoordinates)
					
					yoloTalker(ListeObj2)
			
		###----Extract Ground Truth Data----####
			Object_List_Talker(world,args)


	finally:
		if world is not None:
			world.destroy()

		  

		pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
	argparser = argparse.ArgumentParser(
		description='CARLA Manual Control Client')
	argparser.add_argument(
		'-v', '--verbose',
		action='store_true',
		dest='debug',
		help='print debug information')

	argparser.add_argument(
		'-s', '--visible',
		help='publish only ego data',
		action="store_true")

	argparser.add_argument(
		'--host',
		metavar='H',
		default='127.0.0.1',
		help='IP of the host server (default: 127.0.0.1)')
	argparser.add_argument(
		'-p', '--port',
		metavar='P',
		default=2000,
		type=int,
		help='TCP port to listen to (default: 2000)')
	argparser.add_argument(
		'--res',
		metavar='WIDTHxHEIGHT',
		default='1280x720',
		help='window resolution (default: 1280x720)')
	argparser.add_argument(
		'--filter',
		metavar='PATTERN',
		default='vehicle.*',
		help='actor filter (default: "vehicle.*")')
	argparser.add_argument("-a", "--agent", type=str,
						   choices=["Roaming", "Basic"],
						   help="select which agent to run",
						   default="Basic")
	#args = argparser.parse_args()
	args, unknown = argparser.parse_known_args()

	args.width, args.height = [int(x) for x in args.res.split('x')]

	log_level = logging.DEBUG if args.debug else logging.INFO
	logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

	logging.info('listening to server %s:%s', args.host, args.port)

	print(__doc__)

	try:

		game_loop(args)

	except KeyboardInterrupt:
		print('\nCancelled by user. Bye!')


if __name__ == '__main__':
	main()
