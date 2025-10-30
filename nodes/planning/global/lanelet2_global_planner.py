#!/usr/bin/env python3

import rospy

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path

class Lanelet2GlobalPlanner:
    def __init__(self):

        # Parameters (not sure if they are necessary)
        self.coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        self.use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        self.utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("/localization/utm_origin_lon") # this and upper 3 might also be accessed only once
        self.lanelet2_map_name = rospy.get_param("~lanelet2_map_path") # accessed only once
        self.lanelet2_map = self.load_lanelet2_map() # maybe move upper 5 parameters into this function
        self.output_frame = rospy.get_param("/config/planning/output_frame") # maybe wrong

        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.VehicleTaxi) # might be accessed only once
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, self.traffic_rules)
        self.current_location = None
        self.current_goal = None
        #self.route = None
        
        # Publishers
        self.global_path_pub = rospy.Publisher('/planning/global/global_path', VehicleCmd, queue_size=10, latch=True) # topic might be wrong

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=10)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=10)

        # Messages
        self.global_path_msg = Path()


    def load_lanelet2_map(self):
        # Load the map using Lanelet2
        if self.coordinate_transformer == "utm":
            projector = UtmProjector(Origin(self.utm_origin_lat, self.utm_origin_lon), self.use_custom_origin, False)
        else:
            raise ValueError(
                'Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): ' + self.coordinate_transformer)

        return load(self.lanelet2_map_name, projector)


    def lanelet_to_waypoints(self, route):
        waypoints = []
        for lanelet in route:
            if 'speed_ref' in lanelet.attributes:
                speed = float(lanelet.attributes['speed_ref'])
            else:
                speed = rospy.get_param("~speed_limit") # perhaps have to change the path

            for point in lanelet.centerline:
                waypoint = Waypoint()
                waypoint.speed = speed

                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
            
                waypoints.append(waypoint)

        return waypoints
        
    
    def goal_callback(self, msg):
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                      msg.pose.orientation.w, msg.header.frame_id)
        self.current_goal = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        

    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.current_goal is None:
            return

        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_goal, 1)[0][1]

        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)
        if route is None:
            rospy.logwarn("No route found!")
            return None

        path = route.shortestPath()
        # This returns LaneletSequence to a point where a lane change would be necessary to continue
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        print(f'path_no_lane_change = {path_no_lane_change}')

        waypoints = lanelet_to_waypoints(path_no_lane_change)
        publish_waypoints(waypoints)

    
    def publish_waypoints(self, waypoints):      
        self.global_path_msg.header.frame_id = self.output_frame 
        self.global_path_msg.header.stamp = rospy.Time.now()
        self.global_path_msg.waypoints = waypoints
        self.global_path_pub.publish(self.global_path_msg)
        

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()
