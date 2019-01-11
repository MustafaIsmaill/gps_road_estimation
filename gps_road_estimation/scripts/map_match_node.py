#!/usr/bin/env python

import rospy
import utm
import math

from nav_msgs.msg import Path
from shapely.geometry import Point
from sensor_msgs.msg import NavSatFix
from road_processing_planning.srv import getPath

from map_match import map_match

pub = rospy.Publisher('/ada/fix_corrected', NavSatFix, queue_size=1)
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

def fix_callback(gps):
	global pub
	global map_m

	if isnan(gps.latitude) == False or math.isnan(gps.longitude) == False:
		gps_point = fix_to_utm(gps.latitude, gps.longitude) 
		matched_point, matched_edge = map_m.get_pp_edge(gps_point)
		rospy.loginfo(matched_point)
		pub.publish(gps)
	else:
		rospy.loginfo("gps NaN detected")

if __name__ == '__main__':
	rospy.init_node('map_matching')

	rospy.loginfo("getting path msg ...")
	map_m = map_match(get_path_points())

	rospy.loginfo("ready to subscribe ...")
	sub = rospy.Subscriber(rospy.get_param("/map_matching/sub_topic"), NavSatFix, fix_callback)
    
	rospy.spin()