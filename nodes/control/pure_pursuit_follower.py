#!/usr/bin/env python3

import rospy
import numpy as np

from autoware_mini.msg import Path
from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import VehicleCmd

from shapely.geometry import LineString, Point
from shapely import prepare, distance

from tf.transformations import euler_from_quaternion

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_linestring = None

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10 )

        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

        # Messages
        self.vehicle_cmd_msg = VehicleCmd()
        self.vehicle_cmd_msg.ctrl_cmd.steering_angle = 0.2
        self.vehicle_cmd_msg.ctrl_cmd.linear_velocity = 10.0

    def path_callback(self, msg):
        # convert waypoints to shapely linestring
        path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(path_linestring)
        self.path_linestring = path_linestring

    def current_pose_callback(self, msg):
        #print(f'x: {msg.pose.position.x}, {msg.pose.position.y}')
        self.vehicle_cmd_msg.header.stamp = msg.header.stamp
        self.vehicle_cmd_msg.header.frame_id = "base_link"
        self.vehicle_cmd_pub.publish(self.vehicle_cmd_msg)

        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        if self.path_linestring is not None:
            d_ego_from_path_start = self.path_linestring.project(current_pose)
            print(f'd_ego_from_path_start = {d_ego_from_path_start}')

        # Reading in the parameter values
        lookahead_distance = rospy.get_param("~lookahead_distance")
        wheel_base = rospy.get_param("/vehicle/wheel_base")

        # using euler_from_quaternion to get the heading angle
        _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                               msg.pose.orientation.z, msg.pose.orientation.w])

        # Missing: lookahead point calculation.
        lookahead_point = 0

        # lookahead point heading calculation
        lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
