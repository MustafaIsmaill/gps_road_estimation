#!/usr/bin/env python
# license removed for brevity

import rospy
import roslib
import numpy as np
import tf
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/ada/velodyne_odometry_corrected', Odometry, queue_size=1)
first_x = 0
first_y = 0
first_time = True
br = tf.TransformBroadcaster()


def odom_cb(message):
	global pub
	global first_x
	global first_y
	global first_time
	global br
	initial_or = 216.4/180.0*np.pi  #212.4
	cov_hi = 1000
	cov_lo = 1

	if first_time:
		first_x = message.pose.pose.position.x
		first_y = message.pose.pose.position.y
		first_time = False

	msg = message
	msg.header.frame_id = 'ada/odom'
	temp_x = msg.pose.pose.position.x - first_x
	temp_y = msg.pose.pose.position.y - first_y

	msg.pose.pose.position.x = np.cos(initial_or)*temp_x - np.sin(initial_or)*temp_y
	msg.pose.pose.position.y = np.sin(initial_or)*temp_x + np.cos(initial_or)*temp_y

	roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	new_or = tf.transformations.quaternion_from_euler(roll, pitch, yaw + initial_or)
	msg.pose.pose.orientation.x = new_or[0]
	msg.pose.pose.orientation.y = new_or[1]
	msg.pose.pose.orientation.z = new_or[2]
	msg.pose.pose.orientation.w = new_or[3]

	msg.pose.covariance = [cov_lo, 0.0, 0.0, 0.0, 0.0, 0.0, 
						   0.0, cov_lo, 0.0, 0.0, 0.0, 0.0, 
						   0.0, 0.0, cov_hi, 0.0, 0.0, 0.0, 
						   0.0, 0.0, 0.0, cov_hi, 0.0, 0.0, 
						   0.0, 0.0, 0.0, 0.0, cov_hi, 0.0, 
						   0.0, 0.0, 0.0, 0.0, 0.0, cov_lo]
	msg.twist.covariance = msg.pose.covariance
	pub.publish(msg)

	trans = [msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0]
	br.sendTransform(trans, new_or, rospy.Time.now(), 'ada/rear_wheel_axis', 'ada/odom')
	#print(trans)
	#print(new_or)


if __name__ == '__main__':

    rospy.init_node('vo_fix')
    sub = rospy.Subscriber("/ada/velodyne_odometry", Odometry, odom_cb)
    rospy.spin()


