#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
import threading
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, Log
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from autoware_mini.geometry import project_vector_to_heading, get_distance_between_two_points_2d

class Vector:
    
    def __init__(self, x, y, z):
        
        self.x = x
        self.y = y
        self.z = z

class SpeedPlanner:

    def __init__(self):

        # parameters
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")
        self.distance_to_car_front = rospy.get_param("distance_to_car_front")

        # variables
        self.collision_points = None
        self.current_position = None
        self.current_speed = None

        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)

        ts.registerCallback(self.collision_points_and_path_callback)

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            with self.lock:
                collision_points = numpify(collision_points_msg) if len(collision_points_msg.data) > 0 else np.array([])
                current_position = self.current_position
                current_speed = self.current_speed

            pass

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

        if current_position is None or current_speed is None:
            return

        local_path_linestring = shapely.LineString(
            [w.position.x, w.position.y, w.position.z] for w in local_path_msg.waypoints)
        if len(collision_points) == 0:
            self.local_path_pub.publish(local_path_msg)
            return

        #collision_points_shapely = shapely.points(structured_to_unstructured(collision_points[['x', 'y', 'z']]))
        collision_points_coordinates = []
        for collision_point in collision_points:
            collision_points_coordinates.append([collision_point['x'], collision_point['y'], collision_point['z']])
        collision_points_shapely = shapely.points(collision_points_coordinates)

        collision_point_abs_distances = np.array([local_path_linestring.project(collision_point_shapely) for collision_point_shapely in collision_points_shapely])

        collision_point_heading_angles = np.array([self.get_heading_at_distance(local_path_linestring, distance) for distance in collision_point_abs_distances])
        collision_point_velocity_vectors = np.array([Vector(collision_point['vx'], collision_point['vy'], collision_point['vz']) for collision_point in collision_points])
        collision_point_velocities = np.zeros(len(collision_points))
        for i in range(len(collision_point_velocities)):
            collision_point_velocities[i] = project_vector_to_heading(collision_point_heading_angles[i], collision_point_velocity_vectors[i])
        #collision_point_speeds = np.array((collision_point['vx']**2 + collision_point['vy']**2 + collision_point['vz']**2)**0.5 for collision_point in collision_points) # actual velocity, only here for printing
        
        target_distances = self.braking_reaction_time * abs(collision_point_velocities)
        
        fresh_waypoints = []
        closest_object_distance = 0 # distance between the first waypoint and the collision point closest to it
        smallest_target_velocity_index = 0
        
        # for every waypoint along the local path, find the closest collision point (that is ahead of the waypoint) and corresponding target velocity
        for i, wp in enumerate(local_path_msg.waypoints):
            wp_shapely = shapely.points([wp.position.x, wp.position.y, wp.position.z])
            wp_abs_distance = local_path_linestring.project(wp_shapely) # by abs_distance i mean that this is the distance from the beginning of the local_path_linestring
            
            smallest_target_velocity = 10000
            smallest_target_velocity_index_2 = 0
            closest_collision_point_distance = 10000
            
            for j, collision_point_abs_distance in enumerate(collision_point_abs_distances):
                if collision_point_abs_distance < wp_abs_distance: # the waypoint is past this collision point
                    continue
                    
                collision_point_distance = collision_point_abs_distance - wp_abs_distance # distance from the waypoint to the collision point
                if collision_point_distance < closest_collision_point_distance:
                    closest_collision_point_distance = collision_point_distance

                stopping_distance = max(0, collision_point_distance - self.distance_to_car_front - collision_points[j]['distance_to_stop'] - target_distances[j])
                collision_point_target_velocity = (max(0, collision_point_velocities[j])**2 + 2*self.default_deceleration*stopping_distance)**0.5
                if collision_point_target_velocity < smallest_target_velocity:
                    smallest_target_velocity = collision_point_target_velocity
                    smallest_target_velocity_index_2 = j
                    
            wp.speed = min(smallest_target_velocity, wp.speed)
            
            fresh_waypoints.append(wp)
            
            if i == 0:
                closest_object_distance = collision_point_abs_distances[smallest_target_velocity_index] - wp_abs_distance
                closest_object_velocity = smallest_target_velocity
                stopping_point_distance = stopping_distance
                smallest_target_velocity_index = smallest_target_velocity_index_2

        # Update the lane message with the calculated values
        path = Path()
        path.header = local_path_msg.header
        path.waypoints = fresh_waypoints
        path.closest_object_distance = closest_object_distance # Distance to the collision point with lowest target velocity (also closest object for now)
        path.closest_object_velocity = closest_object_velocity # Velocity of the collision point with lowest target velocity (0)
        path.is_blocked = True
        path.stopping_point_distance = stopping_point_distance # Stopping point distance can be set to the distance to the closest object for now
        path.collision_point_category = collision_points[smallest_target_velocity_index]['category'] # Category of collision point with lowest target velocity
        self.local_path_pub.publish(path)

  
    def get_heading_at_distance(self, linestring, distance):
        """
        Get heading of the path at a given distance
        :param distance: distance along the path
        :param linestring: shapely linestring
        :return: heading angle in radians
        """

        point_after_object = linestring.interpolate(distance + 0.1)
        # if distance is negative it is measured from the end of the linestring in reverse direction
        point_before_object = linestring.interpolate(max(0, distance - 0.1))

        # get heading between two points
        return math.atan2(point_after_object.y - point_before_object.y, point_after_object.x - point_before_object.x)


    def project_vector_to_heading(self, heading_angle, vector):
        """
        Project vector to heading
        :param heading_angle: heading angle in radians
        :param vector: vector
        :return: projected vector
        """

        return vector.x * math.cos(heading_angle) + vector.y * math.sin(heading_angle)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speed_planner')
    node = SpeedPlanner()
    node.run()
