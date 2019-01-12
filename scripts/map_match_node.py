#!/usr/bin/env python

import rospy
import utm
import math

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from road_processing_planning.srv import getPath

from shapely.geometry import Point

from map_match import map_match

gps_pub = rospy.Publisher('/ada/utm_gps', Odometry, queue_size=1)
gps_pp_pub = rospy.Publisher('/ada/gps_p_point', Odometry, queue_size=1)
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

def isnan(value):
	if math.isnan(value) == False:
		return False
	else:
		return True

def fix_to_utm(lat, lon):
	UTMx, UTMy, _, _ = utm.from_latlon(lat, lon)
	return Point(UTMx, UTMy)

def fix_callback(gps_fix):
	global map_m

	if isnan(gps_fix.latitude) == False or math.isnan(gps_fix.longitude) == False:
		gps_point = fix_to_utm(gps_fix.latitude, gps_fix.longitude) 
		matched_point, matched_edge = map_m.get_pp_edge(gps_point)
		publish_msgs(gps_point, matched_point)
	else:
		rospy.loginfo("gps NaN detected")

def publish_msgs(gps_point, matched_point):
	global gps_pub
	global gps_pp_pub

	gps_msg = Odometry()
	pp_msg = Odometry()

	gps_msg.pose.pose.position.x = gps_point.x
	gps_msg.pose.pose.position.y = gps_point.y

	pp_msg.pose.pose.position.x = matched_point.x
	pp_msg.pose.pose.position.y = matched_point.y

	gps_pub.publish(gps_msg)
	gps_pp_pub.publish(pp_msg)

if __name__ == '__main__':
	rospy.init_node('map_matching')

	rospy.loginfo("getting path msg ...")
	map_m = map_match(get_path_points())

	rospy.loginfo("subscribed ...")
	sub = rospy.Subscriber(rospy.get_param("/map_matching/sub_topic"), NavSatFix, fix_callback)
    
	rospy.spin()