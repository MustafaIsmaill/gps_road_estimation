#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/ada/odometry_fusion_ekf_global', Odometry, queue_size=1)
zero_point = Odometry()
count = 1

def fusion_cb(message):
	global pub
	global zero_point
	global count

	if count == 1:
		zero_point = rospy.wait_for_message('/ada/gps_odometry', Odometry)
		count = 2

	message.header.frame_id = 'map'
	message.child_frame_id = 'ada/rear_wheel_axis'

	message.pose.pose.position.x = zero_point.pose.pose.position.x + message.pose.pose.position.x
	message.pose.pose.position.y = zero_point.pose.pose.position.y + message.pose.pose.position.y
	
	pub.publish(message)

if __name__ == '__main__':

    rospy.init_node('add_fusion_offset')
    sub = rospy.Subscriber("/ada/odometry_fusion_ekf", Odometry, fusion_cb)
    rospy.spin()