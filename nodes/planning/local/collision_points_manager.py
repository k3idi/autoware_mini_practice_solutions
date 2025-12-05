#!/usr/bin/env python3

import rospy
import shapely
import math
import numpy as np
import threading
from ros_numpy import msgify
from autoware_mini.msg import Path, DetectedObjectArray
from autoware_mini.msg import TrafficLightResult, TrafficLightResultArray
from sensor_msgs.msg import PointCloud2
from shapely.geometry import LineString
from shapely import prepare, buffer
from shapely import Polygon, intersects, intersection, get_coordinates
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('vx', np.float32),
    ('vy', np.float32),
    ('vz', np.float32),
    ('distance_to_stop', np.float32),
    ('deceleration_limit', np.float32),
    ('category', np.int32)
])

class CollisionPointsManager:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal")
        self.braking_safety_distance_stopline = rospy.get_param("~braking_safety_distance_stopline")
        # Parameters related to lanelet2 map loading
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        lanelet2_map = load(lanelet2_map_path, projector)
        
        # Extract all stop lines and signals from the lanelet2 map
        all_stoplines = self.get_stoplines(lanelet2_map)
        self.trafficlights = self.get_stoplines_trafficlights(lanelet2_map)
        # If stopline_id is not in self.signals then it has no signals (traffic lights)
        self.tfl_stoplines = {k: v for k, v in all_stoplines.items() if k in self.trafficlights}

        # variables
        self.detected_objects = None
        self.goal_waypoint = None
        self.traffick_light_results = None
        
        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_collision_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/detection/traffic_light_status', TrafficLightResultArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)

    def traffic_light_status_callback(self, msg):
        self.traffick_light_results = msg.results  # i don't know what this actually is supposed to be
        
    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def global_path_callback(self, msg):
        if len(msg.waypoints) != 0:
            self.goal_waypoint = msg.waypoints[len(msg.waypoints) - 1]
    
    def path_callback(self, msg):
        with self.lock:
            detected_objects = self.detected_objects
        collision_points = np.array([], dtype=DTYPE)
      
        if len(msg.waypoints) == 0:
            local_path_collision_msg = msgify(PointCloud2, collision_points)
            local_path_collision_msg.header.stamp = msg.header.stamp
            local_path_collision_msg.header.frame_id = msg.header.frame_id
            self.local_path_collision_pub.publish(local_path_collision_msg)
            return # maybe change this to something else
          
        path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
        self.path_linestring = path_linestring
        
        local_path_buffer = buffer(path_linestring, distance=self.safety_box_width/2, cap_style='flat')
        prepare(local_path_buffer) # prepare path - creates spatial tree, making the spatial queries more efficient

        for object in detected_objects:
            object_polygon = Polygon(shell=object.convex_hull)
            if intersects(object_polygon, local_path_buffer):
                geometry_overlap = intersection(object_polygon, local_path_buffer)
                intersection_points = get_coordinates(geometry_overlap)
                for x, y in intersection_points:
                    collision_points = np.append(collision_points, np.array([(x, y, object.centroid.z,
                                                                              object.velocity.x, object.velocity.y, object.velocity.z,
                                                                              self.braking_safety_distance_obstacle, np.inf, 3 if object.speed < self.stopped_speed_limit else 4)], dtype=DTYPE))

        if self.goal_waypoint is not None:
            x, y, z = self.goal_waypoint.position.x, self.goal_waypoint.position.y, self.goal_waypoint.position.z
            collision_points = np.append(collision_points, np.array([(x, y, z, 0, 0, 0, self.braking_safety_distance_goal, np.inf, 1)], dtype=DTYPE))

        if self.traffick_light_results is not None:
            for light in self.traffick_light_results:
                if not (light.recognition_result_str == "red" or light.recognition_result_str == "yellow"):
                    continue

                stopline = self.tfl_stoplines[light.light_id]

                if intersects(stopline, local_path_buffer): # this might always be True
                    geometry_overlap = intersection(stopline, local_path_buffer)
                    intersection_points = get_coordinates(geometry_overlap)
                    for x, y in intersection_points:
                        collision_points = np.append(collision_points, np.array([(x, y, 0, 0, 0, 0, self.braking_safety_distance_stopline, np.inf, 2)], dtype=DTYPE))
        
        local_path_collision_msg = msgify(PointCloud2, collision_points)
        local_path_collision_msg.header.stamp = msg.header.stamp
        local_path_collision_msg.header.frame_id = msg.header.frame_id
        self.local_path_collision_pub.publish(local_path_collision_msg)   

    def get_stoplines(lanelet2_map):
        """
        Add all stop lines to a dictionary with stop_line id as key and stop_line as value
        :param lanelet2_map: lanelet2 map
        :return: {stop_line_id: stopline, ...}
        """
    
        stoplines = {}
        for line in lanelet2_map.lineStringLayer:
            if line.attributes:
                if line.attributes["type"] == "stop_line":
                    # add stoline to dictionary and convert it to shapely LineString
                    stoplines[line.id] = LineString([(p.x, p.y) for p in line])
    
        return stoplines
    
    
    def get_stoplines_trafficlights(lanelet2_map):
        """
        Iterate over all regulatory_elements with subtype traffic light and extract the stoplines and sinals.
        Organize the data into dictionary indexed by stopline id that contains a traffic_light id and the four coners of the traffic light.
        :param lanelet2_map: lanelet2 map
        :return: {stopline_id: {traffic_light_id: {'top_left': [x, y, z], 'top_right': [...], 'bottom_left': [...], 'bottom_right': [...]}, ...}, ...}
        """
    
        signals = {}
    
        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
                linkId = reg_el.parameters["ref_line"][0].id
    
                for tfl in reg_el.parameters["refers"]:
                    tfl_height = float(tfl.attributes["height"])
                    # plId represents the traffic light (pole), one stop line can be associated with multiple traffic lights
                    plId = tfl.id
    
                    traffic_light_data = {'top_left': [tfl[0].x, tfl[0].y, tfl[0].z + tfl_height],
                                          'top_right': [tfl[1].x, tfl[1].y, tfl[1].z + tfl_height],
                                          'bottom_left': [tfl[0].x, tfl[0].y, tfl[0].z],
                                          'bottom_right': [tfl[1].x, tfl[1].y, tfl[1].z]}
    
                    # signals is a dictionary indexed by stopline id and contains dictionary of traffic lights indexed by pole id
                    # which in turn contains a dictionary of traffic light corners
                    signals.setdefault(linkId, {}).setdefault(plId, traffic_light_data)
    
        return signals
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('collision_points_manager')
    node = CollisionPointsManager()
    node.run()
