import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist, PoseWithCovariance, TwistStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from dbw_mkz_msgs.msg import ThrottleCmd,BrakeCmd,SteeringCmd
from std_msgs.msg import UInt8
from tf.transformations import quaternion_from_euler

import carla
import numpy as np
import time
import math
import random

class ROS_Node(object):
    def __init__(self, stage=" ", ns = "carla"):
        self.stage = stage
        self.ns = ns
        self.route = None
        self.throttle = 0
        self.brake = 0
        self.steering = 0
        self.route_planner = self.stage.route_planner
        self.destination = None
        self.path_msg = Path()
        self.waypoints = []
        self.actor_list = self.stage.world.get_actors()
        self.light_list	= self.actor_list.filter("*traffic_light*")
        self.stop_list	= self.actor_list.filter("*stop*")
        self.car_list	= self.actor_list.filter("*vehicle*")
        self.ped_list	= self.actor_list.filter("*walker*")

        rospy.init_node('{}_ros_node'.format(self.ns), anonymous = True)
        rospy.Subscriber('{}/ThrottleCmd'.format(self.ns), ThrottleCmd, self.callback_throttle)
        rospy.Subscriber('{}/BrakeCmd'.format(self.ns), BrakeCmd, self.callback_brake)
        rospy.Subscriber('{}/SteeringCmd'.format(self.ns), SteeringCmd, self.callback_steering)

        # self.callback_throttle(ThrottleCmd())
        self.path_publisher = rospy.Publisher('{}/global_path'.format(self.ns), Path, queue_size = 10)
        self.odom_publisher = rospy.Publisher('{}/odom'.format(self.ns), Odometry, queue_size = 10)
        self.speed_publisher = rospy.Publisher('{}/speed'.format(self.ns), TwistStamped, queue_size = 10)
        self.LiDAR_publisher = rospy.Publisher('{}/LiDAR'.format(self.ns), PointCloud2, queue_size = 10)
        self.left_lanemarking_publisher = rospy.Publisher('{}/left_lanemarking'.format(self.ns), Path, queue_size = 10)
        self.right_lanemarking_publisher = rospy.Publisher('{}/right_lanemarking'.format(self.ns), Path, queue_size = 10)
        self.heading_traffic_light_publisher = rospy.Publisher('{}/heading_traffic_light'.format(self.ns), UInt8, queue_size = 10)
        self.heading_stop_sign_publisher = rospy.Publisher('{}/heading_stop_sign'.format(self.ns), UInt8, queue_size = 10)

        

        return None


    def run_step(self):
        # print("__________")
        # print(self.stage.player.get_transform())
        # print(self.stop_list[0].get_location())
        self.trace_route()
        self.detect_light()
        self.detect_stop()
        self.publish_route()
        self.publish_speed()
        self.publish_odom()
        self.publish_path()
        control_cmd = carla.VehicleControl(throttle=self.throttle,brake=self.brake,steer=self.steering)
        self.stage.player.apply_control(control_cmd)
        return None


    def callback_throttle(self, msg):
        self.throttle=msg.pedal_cmd
        if(self.throttle>0):
            self.brake = 0


    def callback_brake(self, msg):
        self.brake = msg.pedal_cmd
        if(self.brake>0):
            self.throttle = 0

    def callback_steering(self, msg):
        self.steering = - msg.steering_wheel_angle_cmd/8.2


    def publish_speed(self):
        v = self.stage.player.get_velocity()
        twist = TwistStamped()
        twist.twist.linear.x = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        self.speed_publisher.publish(twist)

    def publish_odom(self):
        odom = self.stage.player.get_transform()
        out_odom = Odometry()
        out_odom.pose.pose.position.x = odom.location.x
        out_odom.pose.pose.position.y = -odom.location.y
        out_odom.pose.pose.position.z = odom.location.z

        quat = quaternion_from_euler(odom.rotation.roll/180*np.pi,odom.rotation.pitch/180*np.pi,-odom.rotation.yaw/180*np.pi)
        out_odom.pose.pose.orientation.x = quat[0]
        out_odom.pose.pose.orientation.y = quat[1]
        out_odom.pose.pose.orientation.z = quat[2]
        out_odom.pose.pose.orientation.w = quat[3]

        self.odom_publisher.publish(out_odom)

    def publish_path(self):
        self.path_publisher.publish(self.path_msg)

    def make_path(self):
        self.path_msg = Path()
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.seq = wp.is_intersection
            loc = wp.transform.location
            ori = wp.transform.rotation
            pose.pose.position.x = loc.x
            pose.pose.position.y = -loc.y
            pose.pose.position.z = loc.z
            quat = quaternion_from_euler(ori.roll/180*np.pi,ori.pitch/180*np.pi,-ori.yaw/180*np.pi)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]       
            self.path_msg.poses.append(pose)


    def trace_route(self):
        if self.destination:
            if (self.is_within_distance_ahead(self.destination.transform,self.stage.player.get_transform(),10)):
                self.route = None
                self.waypoints = None
                self.destination  = None
        else:
            self.route = None
            self.waypoints = None
            self.destination = None

        if not self.route:
            start_waypoint = self.stage.map.get_waypoint(self.stage.player.get_location())
            waypoints = self.stage.map.get_topology()
            end_waypoint   = random.choice(waypoints)[0]
            self.route = self.route_planner.trace_route(start_waypoint.transform.location,end_waypoint.transform.location)
            self.waypoints = [] 
            for wp in self.route:
                self.waypoints.append(wp[0])
            self.destination = self.waypoints[-1]
            self.make_path()

        
    def publish_route(self):
        waypoints_in_costmap = []
        for wp in self.waypoints:
            if (self.is_within_distance_ahead(wp.transform,self.stage.player.get_transform(),50)):
                waypoints_in_costmap.append(wp)


        for idx in range(0,len(waypoints_in_costmap),10):
            wp = waypoints_in_costmap[idx]
            self.stage.world.debug.draw_point(wp.transform.location, color=carla.Color(0, 255, 0), life_time=0.2)
            left_marking = self.lateral_shift(wp.transform, -wp.lane_width * 0.5)
            self.stage.world.debug.draw_point(left_marking, color=carla.Color(255, 0, 0), life_time=0.2)
            right_marking = self.lateral_shift(wp.transform, wp.lane_width * 0.5)
            self.stage.world.debug.draw_point(right_marking, color=carla.Color(255, 0, 0), life_time=0.2) 

        left_lanemarking_msg = Path()
        right_lanemarking_msg = Path()
        for wp in waypoints_in_costmap:
            left_marking = self.lateral_shift(wp.transform, -wp.lane_width * 0.5)
            left_marking_pose = PoseStamped()
            left_marking_pose.pose.position.x = left_marking.x
            left_marking_pose.pose.position.y = left_marking.y
            left_marking_pose.pose.position.z = left_marking.z
            left_lanemarking_msg.poses.append(left_marking_pose)
            
            right_marking = self.lateral_shift(wp.transform, wp.lane_width * 0.5)
            right_marking_pose = PoseStamped()
            right_marking_pose.pose.position.x = right_marking.x
            right_marking_pose.pose.position.y = right_marking.y
            right_marking_pose.pose.position.z = right_marking.z
            right_lanemarking_msg.poses.append(right_marking_pose)


        self.left_lanemarking_publisher.publish(left_lanemarking_msg)
        self.right_lanemarking_publisher.publish(right_lanemarking_msg)
        return None


    def lateral_shift(self,transform, shift):
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()


    def is_within_distance_ahead(self,tf1, tf2,max_distance):
        loc1 = tf1.location
        loc2 = tf2.location

        yaw1 = tf1.rotation.yaw
        yaw2 = tf2.rotation.yaw


        distance = np.linalg.norm(np.array([loc1.x-loc2.x,loc1.y-loc2.y,loc1.z-loc2.z]))
        d_angle  = abs((yaw1-yaw2+180)%360-180)

        if distance < 0.1:
            return True
        
        elif distance > max_distance:
            return False

        else:
            return  d_angle< 90.0

    def is_target_light(self,tf1, tf2,max_distance):
        loc1 = tf1.location
        loc2 = tf2.location

        yaw1 = tf1.rotation.yaw
        yaw2 = tf2.rotation.yaw


        distance = np.linalg.norm(np.array([loc1.x-loc2.x,loc1.y-loc2.y,loc1.z-loc2.z]))


        if distance < 0.1:
            return True
        
        elif distance > max_distance: # if the light is close to the car
            return False

        else:
            yaw3 = np.arctan2(loc1.y-loc2.y,loc1.x-loc2.x)
            d_angle1 = abs((yaw3-yaw2+180)%360-180) < 20.0 # if the light is in front of the car
            d_angle2 = abs((yaw1-yaw2+180)%360-180-90) < 20.0 # if the light is facing to the car
            return  d_angle1 and d_angle2


    def detect_light(self):
        tf_ego = self.stage.player.get_transform()
        heading_traffic_light = None
        for light in self.light_list:
            if self.is_target_light(light.get_transform(),tf_ego,60):
                heading_traffic_light = light
                break
        
        state = 0
        if not heading_traffic_light:
            state = 0
        elif(heading_traffic_light.state==carla.TrafficLightState.Green):
            state = 1
        elif(heading_traffic_light.state==carla.TrafficLightState.Yellow):
            state = 2
        elif(heading_traffic_light.state==carla.TrafficLightState.Red):
            state = 3
        self.heading_traffic_light_publisher.publish(state)


    def is_target_stop(self,tf1, tf2,max_distance):
        loc1 = tf1.location
        loc2 = tf2.location

        yaw1 = tf1.rotation.yaw
        yaw2 = tf2.rotation.yaw


        distance = np.linalg.norm(np.array([loc1.x-loc2.x,loc1.y-loc2.y,loc1.z-loc2.z]))


        if distance < 0.1:
            return True
        
        elif distance > max_distance: # if the light is close to the car
            return False

        else:
            # return True
            yaw3 = np.arctan2(loc1.y-loc2.y,loc1.x-loc2.x)
            d_angle1 = abs((yaw3-yaw2+180)%360-180) < 20.0 # if the light is in front of the car
            d_angle2 = abs((yaw1-yaw2+180)%360-180-180) < 20.0 # if the light is facing to the car
            return  d_angle1 and d_angle2

        
    def detect_stop(self):
        tf_ego = self.stage.player.get_transform()
        heading_stop_sign = None
        for stop in self.stop_list:
            if self.is_target_stop(stop.get_transform(),tf_ego,60):
                heading_stop_sign = stop
                # print(stop.get_transform())


        state = 0
        if not heading_stop_sign:
            state = 0
        else:
            state = 1

        self.heading_stop_sign_publisher.publish(state)
