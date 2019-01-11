#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

pub = rospy.Publisher('/ada/speed_odometry', Odometry, queue_size=1)

cov_lo = 1
cov_hi = 1000

def speed_cb(message):
	global pub
	global cov_lo
	global cov_hi

	vehicle_speed = kph_to_mps(message)	
	speed_msg = Odometry()

	speed_msg.header.frame_id = "ada/rear_wheel_axis"

	speed_msg.twist.twist.linear.x = vehicle_speed

	speed_msg.pose.covariance = [cov_hi, 0.0, 0.0, 0.0, 0.0, 0.0, 
						   		 0.0, cov_hi, 0.0, 0.0, 0.0, 0.0, 
						   		 0.0, 0.0, cov_hi, 0.0, 0.0, 0.0, 
						         0.0, 0.0, 0.0, cov_hi, 0.0, 0.0, 
						         0.0, 0.0, 0.0, 0.0, cov_hi, 0.0, 
						         0.0, 0.0, 0.0, 0.0, 0.0, cov_hi]
	speed_msg.twist.covariance = [cov_lo, 0.0, 0.0, 0.0, 0.0, 0.0, 
						   		  0.0, cov_hi, 0.0, 0.0, 0.0, 0.0, 
						   		  0.0, 0.0, cov_hi, 0.0, 0.0, 0.0, 
						          0.0, 0.0, 0.0, cov_hi, 0.0, 0.0, 
						          0.0, 0.0, 0.0, 0.0, cov_hi, 0.0, 
						          0.0, 0.0, 0.0, 0.0, 0.0, cov_hi]

	pub.publish(speed_msg)

def kph_to_mps(speed):
	return speed.data/3.6

if __name__ == '__main__':

    rospy.init_node('speed_odometry')
    sub = rospy.Subscriber("/can/current_speed", Float64, speed_cb)
    rospy.spin()


