#!/usr/bin/env python

import rospy
import utm
import math

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from road_processing_planning.srv import getPath

from shapely.geometry import Point

from map_match import map_match

gps_pp_pub = rospy.Publisher(rospy.get_param("pp_pub_topic"), Odometry, queue_size=1)
map_m = None

def get_path_points():
	rospy.loginfo("Waiting for /path_getter service ...")
	rospy.wait_for_service('path_getter')

	while True:
		try:
			path_getter_service = rospy.ServiceProxy('path_getter', getPath)
			path = path_getter_service(434587,4462624) #### hardcoded goal point
			break
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed: %s Will try again", e)

	path_points = []
	for point in path.path.poses:
		path_point = (point.pose.position.x, point.pose.position.y)
		path_points.append(path_point)

	rospy.loginfo("done parsing path msg ...")
	return path_points

def gps_callback(gps):
	global map_m
	global gps_pp_pub

	gps_point = Point(gps.pose.pose.position.x, gps.pose.pose.position.y)
	matched_point, matched_edge = map_m.get_pp_edge(gps_point)

	gps.pose.pose.position.x = matched_point.x
	gps.pose.pose.position.y = matched_point.y

	gps_pp_pub.publish(gps)

if __name__ == '__main__':
	rospy.init_node('map_matching')

	rospy.loginfo("getting path msg ...")
	map_m = map_match(get_path_points())

	rospy.loginfo("subscribed ...")
	sub = rospy.Subscriber(rospy.get_param("pub_topic"), Odometry, gps_callback)
    
	rospy.spin()