#!/usr/bin/env python3

import rospy
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2

class PointsClusterer:
    def __init__(self):

        # Parameters (not sure if they are necessary)
        self.cluster_epsilon = rospy.get_param("~cluster_epsilon")
        self.cluster_min_size = rospy.get_param("~cluster_min_size")
        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size)
        
        # Publishers
        self.points_clustered_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        labels = self.clusterer.fit_predict(points)

        assert points.shape[0] == labels.shape[0], "there is a different amound of points and labels"

        points_labelled = []
        for i in range(len(points)):
            if labels[i] != -1:
                points_labelled.append((points[i][0], points[i][1], points[i][2], labels[i]))

        points_labelled = np.array(points_labelled)
        data = unstructured_to_structured(points_labelled, dtype=np.dtype([
           ('x', np.float32),
           ('y', np.float32),
           ('z', np.float32),
           ('label', np.int32)
        ]))

        cluster_msg = msgify(PointCloud2, data)
        cluster_msg.header.stamp = msg.header.stamp
        cluster_msg.header.frame_id = msg.header.frame_id

        self.points_clustered_pub.publish(cluster_msg)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('points_clusterer')
    node = PointsClusterer()
    node.run()
