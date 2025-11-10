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


        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        # Messages


    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        print(f'points shape: {points.shape}')

        labels = self.clusterer.fit_predict(points)
        print(f'labels shape: {labels.shape}')

        assert points.shape[0] == labels.shape[0], "there is a different amound of points and labels"
  
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('points_clusterer')
    node = PointsClusterer()
    node.run()
