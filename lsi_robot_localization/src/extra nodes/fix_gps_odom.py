#!/usr/bin/env python

import rospy
import roslib
import numpy as np
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/ada/gps_odometry_corrected', Odometry, queue_size=1)

def odom_callback(message):
	global pub
	
	# f_id = message.header.frame_id
	# f_id = f_id[1:]

	message.child_frame_id = 'ada/rear_wheel_axis'

	pub.publish(message)

if __name__ == '__main__':

    rospy.init_node('gps_correct')
    sub = rospy.Subscriber("/ada/gps_odometry", Odometry, odom_callback)
    rospy.spin()