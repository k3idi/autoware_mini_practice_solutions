#!/usr/bin/env python3

import math
import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer, Proj

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

import time

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('undulation')
        utm_origin_lat = rospy.get_param('utm_origin_lat')
        utm_origin_lon = rospy.get_param('utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.publish_all)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()

        # to publish current pose, velocity and transform
        self.current_pose_msg = PoseStamped()
        self.current_velocity_msg = TwistStamped()
        self.current_transform_msg = TransformStamped()


    # convert azimuth to yaw angle
    def convert_azimuth_to_yaw(self, azimuth):
        """
        Converts azimuth to yaw. Azimuth is CW angle from the north. Yaw is CCW angle from the East.
        :param azimuth: azimuth in radians
        :return: yaw in radians
        """
        yaw = -azimuth + math.pi / 2
        # Clamp within 0 to 2 pi
        if yaw > 2 * math.pi:
            yaw = yaw - 2 * math.pi
        elif yaw < 0:
            yaw += 2 * math.pi

        return yaw

    # publish the position, velocity and transform
    def publish_all(self, msg):
        transformed_x, transformed_y = self.transformer.transform(msg.latitude, msg.longitude)
        transformed_x -= self.origin_x
        transformed_y -= self.origin_y

        azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
        azimuth = math.radians(msg.azimuth - azimuth_correction)
        yaw = self.convert_azimuth_to_yaw(azimuth)

        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(x, y, z, w)

        # publish position
        self.current_pose_msg.header.stamp = msg.header.stamp
        self.current_pose_msg.header.frame_id = "map"
        self.current_pose_msg.pose.position.x = transformed_x
        self.current_pose_msg.pose.position.y = transformed_y
        self.current_pose_msg.pose.position.z = msg.height - self.undulation
        self.current_pose_msg.pose.orientation = orientation
        self.current_pose_pub.publish(self.current_pose_msg)

       # publish transform
        self.current_transform_msg.header.frame_id = "map"
        self.current_transform_msg.child_frame_id = "base_link"
        self.current_transform_msg.header.stamp = msg.header.stamp
        self.current_transform_msg.transform.translation.x = 0 + self.current_pose_msg.pose.position.x
        self.current_transform_msg.transform.translation.y = 0 + self.current_pose_msg.pose.position.y
        self.current_transform_msg.transform.translation.z = 0 + self.current_pose_msg.pose.position.z
        self.current_transform_msg.transform.rotation = self.current_pose_msg.pose.orientation
        self.br.sendTransform(self.current_transform_msg)

        # publish velocity
        self.current_velocity_msg.header.stamp = msg.header.stamp
        self.current_velocity_msg.header.frame_id = "base_link"
        self.current_velocity_msg.twist.linear.x = (msg.north_velocity**2 + msg.east_velocity**2)**0.5
        self.current_velocity_pub.publish(self.current_velocity_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()
