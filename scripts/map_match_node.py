#!/usr/bin/env python

import rospy
import utm
import math

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from road_processing_planning.srv import getPath

from shapely.geometry import Point

from map_match import map_match

goal_status_pub = rospy.Publisher("ada/goal_status", GoalStatusArray, queue_size=1)
gps_pp_pub = rospy.Publisher(rospy.get_param("pp_pub_topic"), Odometry, queue_size=1)

goal_arr = GoalStatusArray()
map_m = None

# ros service
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

	make_goal_arr(path_points)

	return path_points

def make_goal_arr(path_points):
	global goal_arr

	for path in range(0, len(path_points)-1):
		new_goal = GoalStatus()
		
		new_goal.goal_id.id = str(path + 1)
		new_goal.status = 0
		new_goal.text = 'PENDING'

		goal_arr.status_list.append(new_goal)

def final_goal_is_reached(goal_arr):
	status = goal_arr.status_list[len(goal_arr.status_list)-1].status
	if status == 3:
		return True
	else:
		return False

def gps_callback(gps):
	global map_m
	global gps_pp_pub
	global goal_status_pub
	global goal_arr

	gps_point = Point(gps.pose.pose.position.x, gps.pose.pose.position.y)
	
	matched_point, matched_edge = map_m.get_pp_edge(gps_point)
	goal_arr = map_m.update_goal_status(gps_point, matched_edge, goal_arr)

	gps.pose.pose.position.x = matched_point.x
	gps.pose.pose.position.y = matched_point.y

	gps_pp_pub.publish(gps)
	goal_status_pub.publish(goal_arr)

	if final_goal_is_reached(goal_arr) == True:
		rospy.loginfo("reached final goal ...")
		sub.unregister()

if __name__ == '__main__':
	rospy.init_node('map_matching')

	rospy.loginfo("getting path msg ...")
	map_m = map_match(get_path_points())

	rospy.loginfo("subscribed ...")
	sub = rospy.Subscriber(rospy.get_param("pub_topic"), Odometry, gps_callback)
    
	rospy.spin()