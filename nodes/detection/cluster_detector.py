#!/usr/bin/env python3

import rospy
import numpy as np

from shapely import MultiPoint
from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify, msgify

from sensor_msgs.msg import PointCloud2
from autoware_mini.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point32


BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)

class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~min_cluster_size')
        self.output_frame = rospy.get_param('/detection/output_frame')
        self.transform_timeout = rospy.get_param('~transform_timeout')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_clustered', PointCloud2, self.cluster_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())


    def cluster_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        #print(f'points shape before transform: {points.shape}')
        #print(f'first row: {points[0]}')

        if msg.header.frame_id != self.output_frame: # ?
          # fetch transform for target frame
          try:
             transform = self.tf_buffer.lookup_transform(self.output_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(self.transform_timeout))
          except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
             rospy.logwarn("%s - %s", rospy.get_name(), e)
             return
            
        tf_matrix = numpify(transform.transform).astype(np.float32)
        # make copy of points
        points = points.copy()
        # turn into homogeneous coordinates
        points[:,3] = 1
        # transform points to target frame
        points = points.dot(tf_matrix.T)

        #print(f'tf_matrix: {tf_matrix}')
        #print(f'points shape after transform: {points.shape}')

        detected_object_array = DetectedObjectArray()
        detected_object_array.header.stamp = msg.stamp
        detected_object_array.header.frame = self.output_frame
        
        labels = structured_to_unstructured(data['label'], dtype=np.int32) # maybe has to be data[['label']]
        detected_object_array.objects = [] 
        
        if len(labels) == 0: # no clusters
            self.objects_pub.publish(detected_object_array)
            return
        
        for i in range(labels.max() + 1):
            # create mask
            mask = (labels == i)
            # select points for one object from an array using a mask
            # rows are selected using a binary mask, and only the first 3 columns are selected: x, y, and z coordinates
            points3d = points[mask,:3]
            
            if len(points3d) < self.min_cluster_size:
                continue

            centroid = np.mean(points3d, axis=1)

            detected_object = DetectedObject()
            detected_object.centroid.x = centroid[0]
            detected_object.centroid.y = centroid[1]
            detected_object.centroid.z = centroid[2]
            
            # create convex hull
            points_2d = MultiPoint(points[mask,:2])
            hull = points_2d.convex_hull
            convex_hull_points = [a for hull in [[x, y, centroid[2]] for x, y in hull.exterior.coords] for a in hull]
            detected_object.convex_hull = convex_hull_points

            detected_object.id = i
            detected_object.label = "unknown"
            detected_object.color = BLUE80P
            detected_object.valid = True
            detected_object.position_reliable = True
            detected_object.velocity_reliable = False
            detected_object.acceleration_reliable = False

            detected_object_array.objects.append(detected_object)

        self.objects_pub.publish(detected_object_array)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cluster_detector', log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()
