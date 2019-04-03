# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

from __future__ import print_function

import carla
from carla import ColorConverter as cc
from agents.navigation.agent import Agent
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner
from ROS_Node import ROS_Node

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import pygame
import numpy as np

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_actor_display_name(actor, truncate=250):
	name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
	return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
	def __init__(self, carla_world, hud, actor_filter):
		self.world = carla_world
		self.map = self.world.get_map()
		self.hud = hud		# screen #
		self.player = None	# ego-vehicle #
		self.agent  = None	# ego-vehicle -> navigation #
		self.ros_node =  None # ros interface for carla
		self.collision_sensor = None
		self.lane_invasion_sensor = None
		self.gnss_sensor = None
		self.camera_manager = None			# default camera on the screen #
		self._actor_filter = actor_filter 	# select the Lincoln MKZ #
		self.restart()						# initialize the world and the vehicle #
		self.world.on_tick(hud.on_world_tick)	# update the display content #
		self.recording_enabled = False
		self.recording_start = 0
		self.light_list = None
		self.stop_list  = None
		self.car_list	= None
		self.ped_list 	= None
		self.route_planner = None

	def restart(self):
		## Select the default camera for display ##
		cam_index = 0
		cam_pos_index = 0

		## Select the black Lincoln MKZ as ego-vehicle ##
		blueprint = self.world.get_blueprint_library().filter(self._actor_filter)[0]
		blueprint.set_attribute('role_name', 'hero')
		blueprint.set_attribute('color', '50,50,50')
		
		## Spawn the player. ##
		while self.player is None:
			spawn_point = carla.Transform(carla.Location(x=97.2789, y=63.1175, z=1.8431), carla.Rotation(pitch=0, yaw=-10.4166, roll=0))
			self.player = self.world.try_spawn_actor(blueprint, spawn_point)
		print("spawn_point: ",spawn_point)

		## Set up the sensors. ##
		# self.LiDAR_sensor = LiDAR_Sensor(self.player)
		self.collision_sensor = CollisionSensor(self.player, self.hud)
		self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
		self.gnss_sensor = GnssSensor(self.player)
		self.camera_manager = CameraManager(self.player, self.hud)
		self.camera_manager._transform_index = cam_pos_index
		self.camera_manager.set_sensor(cam_index, notify=False)
		actor_type = get_actor_display_name(self.player)
		self.hud.notification(actor_type)

		## Set up agent for scenario perception, route planner and ROS interface
		actor_list = self.world.get_actors()
		self.light_list	= actor_list.filter("*traffic_light*")
		self.stop_list	= actor_list.filter("*stop*")
		self.car_list	= actor_list.filter("*vehicle*")
		self.ped_list	= actor_list.filter("*walker*")
		self.agent = Agent(self.player)
		self.ros_node = ROS_Node(stage=self) 
		dao = GlobalRoutePlannerDAO(self.map)
		self.route_planner = GlobalRoutePlanner(dao)
		self.route_planner.setup()

	def tick(self, clock):
		self.hud.tick(self, clock)

	def render(self, display):
		self.camera_manager.render(display)
		self.hud.render(display)

	def destroy(self):
		actors = [
			self.camera_manager.sensor,
			self.collision_sensor.sensor,
			self.lane_invasion_sensor.sensor,
			self.gnss_sensor.sensor,
			# self.LiDAR_sensor.sensor,
			self.player]
		for actor in actors:
			if actor is not None:
				actor.destroy()


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
	def __init__(self, width, height):
		self.dim = (width, height)
		font = pygame.font.Font(pygame.font.get_default_font(), 20)
		fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
		default_font = 'ubuntumono'
		mono = default_font if default_font in fonts else fonts[0]
		mono = pygame.font.match_font(mono)
		self._font_mono = pygame.font.Font(mono, 14)
		self._notifications = FadingText(font, (width, 40), (0, height - 40))
		# self.help = HelpText(pygame.font.Font(mono, 24), width, height)
		self.server_fps = 0
		self.frame_number = 0
		self.simulation_time = 0
		self._show_info = True
		self._info_text = []
		self._server_clock = pygame.time.Clock()

	def on_world_tick(self, timestamp):
		self._server_clock.tick()
		self.server_fps = self._server_clock.get_fps()
		self.frame_number = timestamp.frame_count
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
		collision = [colhist[x + self.frame_number - 200] for x in range(0, 200)]
		max_col = max(1.0, max(collision))
		collision = [x / max_col for x in collision]
		vehicles = world.world.get_actors().filter('vehicle.lin*')
		self._info_text = [
			'Server:  % 16.0f FPS' % self.server_fps,
			'Client:  % 16.0f FPS' % clock.get_fps(),
			'',
			'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
			'Map:     % 20s' % world.map.name,
			'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
			'',
			'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
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
				'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
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
			distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
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
				if item: # At this point has to be a str.
					surface = self._font_mono.render(item, True, (255, 255, 255))
					display.blit(surface, (8, v_offset))
				v_offset += 18
		self._notifications.render(display)


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
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
	def __init__(self, parent_actor, hud):
		self.sensor = None
		self._history = []
		self._parent = parent_actor
		self._hud = hud
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.collision')
		self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

	def get_collision_history(self):
		history = collections.defaultdict(int)
		for frame, intensity in self._history:
			history[frame] += intensity
		return history

	@staticmethod
	def _on_collision(weak_self, event):
		self = weak_self()
		if not self:
			return
		actor_type = get_actor_display_name(event.other_actor)
		self._hud.notification('Collision with %r' % actor_type)
		impulse = event.normal_impulse
		intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
		self._history.append((event.frame_number, intensity))
		if len(self._history) > 4000:
			self._history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
	def __init__(self, parent_actor, hud):
		self.sensor = None
		self._parent = parent_actor
		self._hud = hud
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.lane_detector')
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
		text = ['%r' % str(x).split()[-1] for x in set(event.crossed_lane_markings)]
		self._hud.notification('Crossed line %s' % ' and '.join(text))

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
		self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
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
# -- LiDAR Sensor --------------------------------------------------------
# ==============================================================================


class LiDAR_Sensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self._parent = parent_actor
		self.points = None
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
		bp.set_attribute('channels','16')
		bp.set_attribute('range','8000')
		bp.set_attribute('upper_fov','15.0')
		bp.set_attribute('lower_fov','-15.0')
		bp.set_attribute('sensor_tick','0.1')
		# bp.set_attribute('points_per_second','36000')
		self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1, z=1.7)), attach_to=self._parent)
		# print(self.position)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: LiDAR_Sensor._on_obst_event(weak_self, event))

	@staticmethod
	def _on_obst_event(weak_self, lidar_measurement):
		self = weak_self()
		if not self:
			return
		# print(lidar_measurement[0].x)
		if len(lidar_measurement):
			self.points = lidar_measurement
		else:
			self.points = None
		# self.lat = event.latitude
		# self.lon = event.longitude

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
	def __init__(self, parent_actor, hud):
		self.sensor = None
		self._surface = None
		self._parent = parent_actor
		self._hud = hud
		self._recording = False
		self._camera_transforms = [
			carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
			carla.Transform(carla.Location(x=1.6, z=1.7))]
		self._transform_index = 1
		self._sensors = [
			['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
			['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
			['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
			['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
			['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
			['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
			['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
		world = self._parent.get_world()
		bp_library = world.get_blueprint_library()
		for item in self._sensors:
			bp = bp_library.find(item[0])
			if item[0].startswith('sensor.camera'):
				bp.set_attribute('image_size_x', str(hud.dim[0]))
				bp.set_attribute('image_size_y', str(hud.dim[1]))
			elif item[0].startswith('sensor.lidar'):
				bp.set_attribute('range', '5000')
			item.append(bp)
		self._index = None

	def toggle_camera(self):
		self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
		self.sensor.set_transform(self._camera_transforms[self._transform_index])

	def set_sensor(self, index, notify=True):
		index = index % len(self._sensors)
		needs_respawn = True if self._index is None \
			else self._sensors[index][0] != self._sensors[self._index][0]
		if needs_respawn:
			if self.sensor is not None:
				self.sensor.destroy()
				self._surface = None
			self.sensor = self._parent.get_world().spawn_actor(
				self._sensors[index][-1],
				self._camera_transforms[self._transform_index],
				attach_to=self._parent)
			# We need to pass the lambda a weak reference to self to avoid
			# circular reference.
			weak_self = weakref.ref(self)
			self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
		if notify:
			self._hud.notification(self._sensors[index][2])
		self._index = index

	def next_sensor(self):
		self.set_sensor(self._index + 1)

	def toggle_recording(self):
		self._recording = not self._recording
		self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

	def render(self, display):
		if self._surface is not None:
			display.blit(self._surface, (0, 0))

	@staticmethod
	def _parse_image(weak_self, image):
		self = weak_self()
		if not self:
			return
		if self._sensors[self._index][0].startswith('sensor.lidar'):
			points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
			points = np.reshape(points, (int(points.shape[0]/3), 3))
			lidar_data = np.array(points[:, :2])
			lidar_data *= min(self._hud.dim) / 100.0
			lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
			lidar_data = np.fabs(lidar_data)
			lidar_data = lidar_data.astype(np.int32)
			lidar_data = np.reshape(lidar_data, (-1, 2))
			lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
			lidar_img = np.zeros(lidar_img_size)
			lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
			self._surface = pygame.surfarray.make_surface(lidar_img)
		else:
			image.convert(self._sensors[self._index][1])
			array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
			array = np.reshape(array, (image.height, image.width, 4))
			array = array[:, :, :3]
			array = array[:, :, ::-1]
			self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
		if self._recording:
			image.save_to_disk('_out/%08d' % image.frame_number)


def main_loop(args):
	pygame.init()
	pygame.font.init()
	world = None

	try:
		client = carla.Client(args.host, args.port)
		client.set_timeout(2.0)

		display = pygame.display.set_mode(
			(args.width, args.height),
			pygame.HWSURFACE | pygame.DOUBLEBUF)

		hud = HUD(args.width, args.height)
		world = World(client.get_world(), hud, args.filter)

		clock = pygame.time.Clock()
		while True:
			world.ros_node.run_step()
			clock.tick_busy_loop(20)
			world.tick(clock)
			world.render(display)
			pygame.display.flip()

	finally:

		if (world and world.recording_enabled):
			client.stop_recorder()

		if world is not None:
			world.destroy()

		pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
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
		'-a', '--autopilot',
		action='store_true',
		help='enable autopilot')
	argparser.add_argument(
		'--res',
		metavar='WIDTHxHEIGHT',
		default='1280x720',
		help='window resolution (default: 1280x720)')
	argparser.add_argument(
		'--filter',
		metavar='PATTERN',
		default='vehicle.lin*',
		help='actor filter (default: "vehicle.lin*")')
	args = argparser.parse_args()

	args.width, args.height = [int(x) for x in args.res.split('x')]

	log_level = logging.DEBUG if args.debug else logging.INFO
	logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

	logging.info('listening to server %s:%s', args.host, args.port)

	try:
		main_loop(args)
	except KeyboardInterrupt:
		print('\nCancelled by user. Bye!')


if __name__ == '__main__':

	main()