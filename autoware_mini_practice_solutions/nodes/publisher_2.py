#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

rospy.init_node('publisher')
freq = rospy.get_param('~frequency', 'Hello World!')
rate = rospy.Rate(int(freq))
pub = rospy.Publisher('/message', String, queue_size=10)

while not rospy.is_shutdown():
    message = rospy.get_param('~message', 'Hello World!')
    pub.publish(message)
    rate.sleep()
