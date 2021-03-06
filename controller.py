#!/usr/bin/env python

from math import *
import numpy as np
import time
import sys
# import cutils

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from ackermann_msgs.msg import AckermannDrive

import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

global debug
debug = 0
class closestPoint:
	def __init__(self, points):
		self.points = points

	def distance(self,p1,p2):
		return sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)

	def closest_node(self, node):
		nodes = self.points
		# print nodes
		print node
		# if debug:
		# 	print nodes
			# logging.debug("Points type: {}".format(type(nodes)))
			# logging.debug("Points array {0}\n".format(nodes.size()))
			# logging.debug("Node type: {0} content: {1} \n".format(type(node),node))
		try:
			dist_2 = np.sum((nodes - node)**2, axis=1)
		except TypeError:
			dist_2 = np.sum((nodes - (0,0))**2, axis=1)
		return np.argmin(dist_2)

class purePursuit:

	def __init__(self,points):

		rospy.loginfo("initializing pure pursuit node...")
		time.sleep(0.5)
		# rospy.init_node("purePursuit_node",anonymous=True)

		self.look_ahead_distance = 5

		self.target_speed = 5

		self.k = 0.
		self.kp = 0.2

		self.init_xy = False
		self.init_heading = False

		self.car_init_x = None
		self.car_init_y = None 
		self.car_init_heading = None

		self.car_current_x = None
		self.car_current_y = None 
		self.car_current_heading = None
		self.current_speed = None
		
		# Get vehicle location from CARLA

		# Initialize vehicle controller message
		# data = rospy.wait_for_message("/carla/ego_vehicle/odometry", Odometry)
		# print data
		rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.callback_odom)
		self.cmd_pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)

		self.goalxpub = rospy.Publisher('/goalx',Float64, queue_size=10)
		self.goalypub = rospy.Publisher('/goaly',Float64, queue_size=10)

		self.points = points
		

	def callback_odom(self,data):
		
		# rospy.loginfo(__func__)
		self.car_current_x = data.pose.pose.position.x
		self.car_current_y = data.pose.pose.position.y

		current_speed_x = data.twist.twist.linear.x
		current_speed_y = data.twist.twist.linear.y
		self.current_speed = np.sqrt(current_speed_x**2 + current_speed_y**2)

		quaternion = [data.pose.pose.orientation.x,\
					   data.pose.pose.orientation.y,\
					   data.pose.pose.orientation.z,\
					   data.pose.pose.orientation.w]
		euler = euler_from_quaternion(quaternion)
		# euler = tf.transformations.euler_from_quaternion(quaternion)
		self.car_current_heading = euler[2]


	def _controller(self):

		############################## FIX #########################################
		# data = rospy.wait_for_message("/carla/ego_vehicle/odometry", Odometry)

		# # print data

		# self.car_current_x = data.pose.pose.position.x
		# self.car_current_y = data.pose.pose.position.y

		# current_speed_x = data.twist.twist.linear.x
		# current_speed_y = data.twist.twist.linear.y
		# self.current_speed = np.sqrt(current_speed_x**2 + current_speed_y**2)

		# quaternion = [data.pose.pose.orientation.x,\
		# 			   data.pose.pose.orientation.y,\
		# 			   data.pose.pose.orientation.z,\
		# 			   data.pose.pose.orientation.w]
		# euler = euler_from_quaternion(quaternion)
		# # euler = tf.transformations.euler_from_quaternion(quaternion)
		# self.car_current_heading = euler[2]
		#############################################################################

		path = closestPoint(self.points)
		ind = path.closest_node((self.car_current_x,self.car_current_y))

		L = 0
		dind = ind
		print "current speed {0} m/s (from twist)".format(self.current_speed)
		Lf = self.k*self.current_speed + self.look_ahead_distance
		while L<Lf and dind < self.points.shape[0]-1:
			dx = self.points[dind+1][0] - self.points[dind][0]
			dy = self.points[dind+1][1] - self.points[dind][1]

			d = sqrt(dx**2 + dy**2)

			L += d

			dind += 1


		goal_x = self.points[(dind)%len(self.points)][0]
		goal_y = self.points[(dind)%len(self.points)][1]
		near_x = self.points[(ind)%len(self.points)][0]
		near_y = self.points[(ind)%len(self.points)][1]

		self.goalxpub.publish(goal_x)
		self.goalypub.publish(goal_y)

		distance = sqrt((self.car_current_x-goal_x)**2 + (self.car_current_y-goal_y)**2)
		distance_final = sqrt((self.car_current_x-self.points[-1][0])**2 + (self.car_current_y-self.points[-1][1])**2)
		distance2 = sqrt((near_x-goal_x)**2 + (near_y-goal_y)**2)
		print "distance_final : ",distance_final

		desired_pose = atan2((goal_y-self.car_current_y),(goal_x-self.car_current_x))

		error = (desired_pose - self.car_current_heading)

		if debug:
			print "Car current position :: ",self.car_current_x,self.car_current_y
			print "Goal position :: ",goal_x,goal_y
			print "Nearest point on the path:: ",near_x,near_y
			print "Distance to goal :: ",distance
			print "Look ahead distance:: ",distance2
			print "desired_pose:: {0} ({1}) ".format(desired_pose,degrees(desired_pose))
			print "current heading:: {0} ({1}) ".format(self.car_current_heading,degrees(self.car_current_heading))
			print "heading error:: {0} ({1}) ".format(error,degrees(error))

		# if error < -3.14159:
		# 	# print "error is below -pi"
		# 	error += 2*3.14159
		# elif error > 3.14159:
		# 	# print "error is above pi"
		# 	error -= 2*3.14159
		# else:
		# 	# print "error is in between -pi and pi"
		# 	pass

		L = 2.6
		ld = 10
		kl = 1
		# self.throttle = self._PIDControl(self.target_speed,self.current_speed)
		# self.throttle = min(self.throttle,0.2)
		if(distance_final < 1):
			self.throttle = 0;
		self.throttle = 5;
		# self.steering = np.arctan(2*L*sin(error)/(kl*self.throttle))
		self.steering_ratio = 1
		self.steering = self.steering_ratio * np.arctan2(2*L*sin(error),Lf)
		# self.steering = error


	def _PIDControl(self,target, current):
		print "\n"
		print "_PIDControl: ", target-current
		print "\n"
		a = self.kp * (target - current)
		return a

	def publish_(self):

		self._controller()

		ai = self.throttle
		di = self.steering 

		# print "throttle and steering angle :: ",di
		print "speed and steering angle :: ",ai,di
		print "\n\n"

		ackermann_cmd_msg = AckermannDrive()
		ackermann_cmd_msg.steering_angle = di 
		ackermann_cmd_msg.speed = ai 

		self.cmd_pub.publish(ackermann_cmd_msg)
		# throttle_msg = make_throttle_msg(ai)
		# steering_msg = make_steering_msg(di)
		# brake_msg = make_brake_msg()
		# self.throttle_pub.publish(throttle_msg)
		# self.steering_pub.publish(steering_msg)
		# self.brake_pub.publish(brake_msg)

		rospy.on_shutdown(self.stopOnShutdown)
	def stopOnShutdown(self):
		ai = 0
		di = 0

		ackermann_cmd_msg = AckermannDrive()
		ackermann_cmd_msg.steering_angle = di 
		ackermann_cmd_msg.speed = ai 

		self.cmd_pub.publish(ackermann_cmd_msg)

		# time.sleep(1)
		# reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
		# reset_world()
		# print "exiting..."
		rospy.signal_shutdown('Quit')
		# print "is shutdown :: ",rospy.is_shutdown()
		# return

def main():
	global points

	controller_object = purePursuit(points)

	r = rospy.Rate(50)

	while not rospy.is_shutdown():
		controller_object.publish_()
		r.sleep()

if __name__ == '__main__':
	main()