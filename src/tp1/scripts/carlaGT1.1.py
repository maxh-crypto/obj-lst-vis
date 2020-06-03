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
from object_list.msg import ObjectsList
from object_list.msg import ObjectList
from object_list.msg import Geometric as geo


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
		#spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform() 
			spawn_point = carla.Transform(carla.Location(x=119.600000, y=8.330196, z=1.843102), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
			self.player = self.world.spawn_actor(blueprintPlayer, spawn_point)
			self.vehicle_control = carla.VehicleControl()
			self.vehicle_control.throttle = 1.0
			
			#self.player.set_velocity(carla.Vector3D(x=11.111111, y=0.000000, z=0.000000)) #40kmh
	   
		# Spawn the npc cars
			spawn_point = carla.Transform(carla.Location(x=219.600000, y=12.800000, z=1.843102), carla.Rotation(pitch=0.000000, yaw=0.855823, roll=0.000000))
			self.npc1 = self.world.try_spawn_actor(blueprintNpc1, spawn_point)
			#self.npc1.set_velocity(carla.Vector3D(x=4.000000, y=0.000000, z=0.000000))
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
			
   		# Set up walker control by AI (not used in this scenario)
			#self.walker1.controller = self.world.spawn_actor(walker_controller_bp, carla.Transform(), self.walker1)
		#self.walker1.controller.start()
			#self.walker1.controller.go_to_location(carla.Location(x=151.516594, y=49.308423, z=1.863102))
		#self.walker1.controller.set_max_speed(1.6)
		
			# Set up walker control by direct input 
		self.control = carla.WalkerControl()
		self.control.speed = 1.39
		self.control.direction = carla.Vector3D(x=0.000000, y=-1.000000, z=0.000000)
		
		
		
		npc1_dim = Dimension(self.npc1)
		#print(npc1_dim.length)
		print(self.player.bounding_box.extent)
		print(self.npc1.bounding_box.extent)
		print(self.npc2.bounding_box.extent)
		print(self.walker1.bounding_box.extent)
			
		# Set up the sensors.
		self.collision_sensor = CollisionSensor(self.player, self.hud)
		self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
		self.gnss_sensor = GnssSensor(self.player)
		self.camera_manager = CameraManager(self.player, self.hud)
		self.camera_manager.transform_index = cam_pos_index
		self.camera_manager.set_sensor(cam_index, notify=False)
		actor_type = get_actor_display_name(self.player)
		self.hud.notification(actor_type)
		#self.LOS_sensor = LineOfSightSensor(self.npc2,self.hud)

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
	
	



	def ground_truth(self):					 #calculates coordinates relative to player
		actors = [
			self.npc1,
			self.npc2,
			self.walker1]
		self.player_loc = -self.player.get_location() - self.player.get_location()
		self.npc1_loc = self.player.get_location() - self.npc1.get_location()
		self.npc2_loc = self.player.get_location() - self.npc2.get_location()
		self.walker1_loc = self.player.get_location() - self.walker1.get_location()
		
   
	def destroy(self):
		actors = [
			self.camera_manager.sensor,
			self.collision_sensor.sensor,
			self.lane_invasion_sensor.sensor,
			self.gnss_sensor.sensor,
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

		FR = actor.bounding_box.extent
		FR.x = 0
		FR.z = 0
		FR = FR + (actor.get_location() - world.player.get_location())
		angle = math.atan2(FR.y, FR.x) * 180 / math.pi
		if FR.x>0 and FR.y<0 and FR.x<=200 and abs(angle) < fov_angle:
			FRbool = 1
		else:
			FRbool = 0
	
		FM = actor.bounding_box.extent
		FM.x = 0
		FM.z = 0 
		FM.y = 0
		FM = FM + (actor.get_location() - world.player.get_location())
		angle = math.atan2(FM.y, FM.x) * 180 / math.pi
		if FM.x>0 and FM.x<=200 and abs(angle) < fov_angle:
			FMbool = 0
		else:
			FMbool = 0
	
		FL = actor.bounding_box.extent
		FL.x = 0
		FL.z = 0
		FL.y = -FL.y
		FL = FL + (actor.get_location() - world.player.get_location())
		angle = math.atan2(FL.y, FL.x) * 180 / math.pi
		if FL.x>0 and FL.y>0 and FL.x<=200 and abs(angle) < fov_angle:
			FLbool = 1
		else:
			FLbool = 0

		
		MR = actor.bounding_box.extent
		MR.x = -MR.x
		MR.z = 0
		MR = MR + (actor.get_location() - world.player.get_location())
		angle = math.atan2(MR.y, MR.x) * 180 / math.pi
		if MR.x>0 and MR.y<0 and MR.x<=200 and abs(angle) < fov_angle:
			MRbool = 1
		else:
			MRbool = 0
	

	
		ML = actor.bounding_box.extent
		ML.x = -ML.x
		ML.z = 0
		ML.y = -ML.y
		ML = ML + (actor.get_location() - world.player.get_location())	
		angle = math.atan2(ML.y, ML.x) * 180 / math.pi
		if ML.x>0 and ML.y>0 and ML.x<=200 and abs(angle) < fov_angle:
			MLbool = 1
		else:
			MLbool = 0
	
		RR = actor.bounding_box.extent
		RR.x = -2*RR.x
		RR.z = 0
		RR = RR + (actor.get_location() - world.player.get_location())	
		angle = math.atan2(RR.y, RR.x) * 180 / math.pi	
		if RR.x>0 and RR.x<=200 and abs(angle) < fov_angle:
			RRbool = 1
		else:
			RRbool = 0
	
		RM = actor.bounding_box.extent
		RM.x = -2*RM.x
		RM.z = 0
		RM.y = 0
		RM = RM + (actor.get_location() - world.player.get_location())
		angle = math.atan2(RM.y, RM.x) * 180 / math.pi
		if RM.x>0 and RM.x<=200 and abs(angle) < fov_angle:
			RMbool = 1
		else:
			RMbool = 0
	
		RL = actor.bounding_box.extent
		RL.x = -2*RL.x
		RL.z = 0
		RL.y = -RL.y
		RL = RL + (actor.get_location() - world.player.get_location())	
		angle = math.atan2(RL.y, RL.x) * 180 / math.pi

		if RL.x>0 and RL.x<=200 and abs(angle) < fov_angle:
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
		self.check_exist()
	
	def check_exist(self):
		val = self.FR +self.FM +self.FL +self.MR +self.ML +self.RR +self.RM +self.RL
		if val > 0:
			self.exist = 1
		else:
			self.exist = 0
		

class Dimension:
	def __init__(self,actor):
		self.length = actor.bounding_box.extent.x *2
		self.height = actor.bounding_box.extent.z *2
		self.width = actor.bounding_box.extent.y *2

class Geometric:
	def __init__(self,world,actor):
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

# ab hier werden die Eigenschaften eines Objektes zugewiesen
	 
	Npc1_geo = Geometric(world,world.npc1)
	Npc1_dim = Dimension(world.npc1)
	Npc1_fea = Feature(world,world.npc1)
	Npc2_geo = Geometric(world,world.npc2)
	Npc2_dim = Dimension(world.npc2)
	Npc2_fea = Feature(world,world.npc2)
	Walker1_geo = Geometric(world,world.walker1)
	Walker1_dim = Dimension(world.walker1)
	Walker1_fea = Feature(world,world.walker1)
	Player_geo = Geometric(world,world.npc1)

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



	

# der ObjektListe werden noch charakteristische Eigenschaften zugewiesen wie			# TimeStamp und Frame ID

#b.header.stamp = rospy.Time.now()
	b.header.frame_id = "ObjectList_GroundTruth"


	

# hier werden die Objekte der Objektliste hinzugefuegt mittels append
	if Npc2_fea.exist == 1 or args.visible:
		b.obj_list.append(a2)
	if Npc1_fea.exist == 1 or args.visible:
		b.obj_list.append(a1)
	if Walker1_fea.exist == 1 or args.visible:
		b.obj_list.append(a3)
	b.header.stamp = rospy.Time.now()
# hier wird die Nachricht gepublished und ein Node kann diese Nachricht abonnieren
	if Npc1_fea.exist == 1 or Npc2_fea.exist == 1 or Walker1_fea.exist == 1:
		pub.publish(b)
	#rospy.loginfo(b)
	

	pub = rospy.Publisher('egovehicle', ObjectsList, queue_size=100) #
	rospy.init_node('camera',anonymous=False)  # Initiate the node camera and anonymous true permitt openinig this node a lot of time including number in the end of the node name  
   #rate=rospy.Rate(50)  #50 hz

   #while not rospy.is_shutdown():

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

	try:
		client = carla.Client(args.host, args.port)
		client.set_timeout(4.0)

		display = pygame.display.set_mode(
			(args.width, args.height),
			pygame.HWSURFACE | pygame.DOUBLEBUF)

		hud = HUD(args.width, args.height)
		world = World(client.get_world(), hud, args.filter)
		controller = KeyboardControl(world)

#		if args.agent == "Roaming":
 #		   agent = RoamingAgent(world.player)
  #	  else:
   #		 agent = BasicAgent(world.player)
	#		spawn_point = world.map.get_spawn_points()[0]
	 #	   agent.set_destination((spawn_point.location.x,wait_for_tick
	  #							 spawn_point.location.y,
	   #							spawn_point.location.z))
	   #World.player.set_velocity(carla.Vector3D(x=4.000000, y=0.000000, z=0.000000))

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
			#if player_pos.x > 219:
			#	world.player.set_velocity(carla.Vector3D(x=0.000000, y=0.000000, z=0.000000))
			#else:
			#	world.player.set_velocity(carla.Vector3D(x=11.10987162, y=0.165959, z=0.000000))
			
			player_velocity = world.player.get_velocity()
			player_velocity = math.sqrt(player_velocity.x ** 2 + player_velocity.y ** 2)
		
			print(player_velocity * 3.6)
			
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
	
			
		###----Extract Ground Truth Data----####
			Object_List_Talker(world,args)
			#Object_List_Talker_Player(world,args)
			
			#if args.ego_data:
				#print("nice")


		#control.agent.run_step()
			#control.manual_gear_shift = False
			#world.player.apply_control(control)

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
	args = argparser.parse_args()

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
