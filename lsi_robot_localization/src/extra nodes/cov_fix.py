#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/ada/velodyne_odometry_corrected', Odometry, queue_size=1)

def odom_cb(message):
	global pub
	
	cov_1 = 1000
	cov_2 = 0.1

	message.pose.covariance = [cov_1, 0.0, 0.0, 0.0, 0.0, 0.0, 
	                           0.0, cov_1, 0.0, 0.0, 0.0, 0.0, 
	                           0.0, 0.0, cov_1, 0.0, 0.0, 0.0, 
	                           0.0, 0.0, 0.0, cov_2, 0.0, 0.0,
	                           0.0, 0.0, 0.0, 0.0, cov_2, 0.0, 
	                           0.0, 0.0, 0.0, 0.0, 0.0, cov_2]

	message.twist.covariance = message.pose.covariance
	pub.publish(message)

if __name__ == '__main__':

    rospy.init_node('vo_fix')
    sub = rospy.Subscriber("/ada/velodyne_odometry", Odometry, odom_cb)
    rospy.spin()