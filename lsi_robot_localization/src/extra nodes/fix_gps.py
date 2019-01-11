#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

pub = rospy.Publisher('/ada/fix_corrected', NavSatFix, queue_size=1)

def fix_callback(message):
	global pub
	
	message.header.frame_id = 'utm'

	pub.publish(message)

if __name__ == '__main__':

    rospy.init_node('fix_correct')
    sub = rospy.Subscriber("/ada/fix", NavSatFix, fix_callback)
    rospy.spin()