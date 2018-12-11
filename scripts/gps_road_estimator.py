#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import pandas as pd
import geopandas
import time
import utm

from shapely.geometry import Point
from shapely.geometry import LineString

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from road_processing_planning.srv import getPath

class estimate_road:
	def __init__(self):
		self._path_points = [] # used to store the coordinates of way points on the path
		self._matched_points = [] # used to store the projected gps points on the path
		self._prev_matched_edge = None # attribute used for comparison between path segments
		self._max_segment_length = 0.0 # attribute used to store the length of the longest path segment

		rospy.init_node('gps_road_estimator', anonymous=True) # initialize ros node

		self._gps_subscriber = rospy.Subscriber("/fix", NavSatFix, self._gps_callback) # create gps sensor subscriber

	# gps callback function
	# this function will be called with every new gps msg received
	def _gps_callback(self, gps):
		# convert gps latitude and longitude to utm coordinates
		UTMx, UTMy, _, _ = utm.from_latlon(gps.latitude, gps.longitude)
		# create a shapely geometry point object for the gps point
		gps_point = Point(UTMx, UTMy)
		# match the gps point to path
		matched_point, matched_edge = self._map_match(gps_point)

		# add received projected point to list
		# this list can be used later for plotting
		self._matched_points.append(matched_point)

		# compare the current path segment with previous
		# if new path segment is detected this means that the vehicle has passed the previous way point
		if self._prev_matched_edge != None:
			if matched_edge != self._prev_matched_edge:
				print("### way point checked ###")

		# next way point is the last point on the current edge
		next_wp = matched_edge.coords[:][len(matched_edge.coords[:])-1]
		# calculate the distance to next waypoint
		distance_to_next_wp = matched_point.distance(Point(next_wp[0], next_wp[1]))

		# the current path segment is now the previous road segment
		# as a new path segment is received everytime this function is called
		self._prev_matched_edge = matched_edge

		# print("done matching %i..." % gps.header.seq)
		print("distance to next way point is %f" % distance_to_next_wp)

		# uncomment the following section for plotting
		# if gps.header.seq == 4169:
		# 	_, self._axis = plt.subplots()
		# if gps.header.seq == 4534: #4534
		# 	self._gps_subscriber.unregister()
		# 	self._plot_path()
		# 	self._plot_matching_output()
		# 	plt.show()

	# this function takes a gps point as input and returns its projection on the nearest path segment 
	# and returns the start and end points of this path segment as well
	def _map_match(self, gps_point):

	    circle = gps_point.buffer(self._max_segment_length/2.0)
	    possible_matches_index = list(self._nodes_spatial_index.intersection((circle.bounds)))
	    possible_matches = self._nodes_gdf.iloc[possible_matches_index]
	    precise_matches = possible_matches[possible_matches.intersects(circle)]
	    candidate_nodes = list(precise_matches.index)

	    candidate_edges = []
	    for node_id in candidate_nodes:
	        if node_id == len(self._nodes_gdf)-1:
	            point_tuple_in = (node_id-1, node_id)
	            candidate_edges.append(point_tuple_in)
	        else:
	            point_tuple_in = (node_id-1, node_id)
	            point_tuple_out = (node_id, node_id+1)
	            candidate_edges.append(point_tuple_in)
	            candidate_edges.append(point_tuple_out)

	    distance = []
	    length = []
	    for edge in candidate_edges:
	        line_string = self._edges_gdf[(self._edges_gdf.u == edge[0]) & (self._edges_gdf.v == edge[1])].geometry
	        distance.append([line_string.distance(gps_point), edge, line_string])

	        d = line_string.distance(gps_point)
	        length.append(d.iloc[0])

	    _, idx = min((length[i],i) for i in xrange(len(length)))
	    true_edge = distance[idx][1]
	    true_edge_geom = distance[idx][2].item()
	    projected_point = true_edge_geom.interpolate(true_edge_geom.project(gps_point)) # projected point

	    return projected_point, true_edge_geom

	def _get_path_points(self): #from service
		self._path = Path()

		rospy.wait_for_service('path_getter')

		try:
			self._path_getter_service = rospy.ServiceProxy('path_getter', getPath)
			self._path = self._path_getter_service('Universidad Carlos III de Madrid, 30, Avenida de la Universidad')
		except rospy.ServiceException, e:
			print("Service call failed: %s"%e)

		for point in self._path.path.poses:
			path_point = (point.pose.position.x, point.pose.position.y)
			self._path_points.append(path_point)

		print("done getting path points ...")

		self._get_path_edges()
		self._get_path_nodes()

		return self._path_points
		
	def _get_path_edges(self):
		path_edges = []
		data = {'id': [], 'u': [], 'v': [], 'geometry': []}

		for i in range(0, len(self._path_points)-1):
			path_edges.append([(self._path_points[i][0], self._path_points[i][1]), (self._path_points[i+1][0], self._path_points[i+1][1])])

		for i in range(0, len(path_edges)):
			data['u'].append(i)
			data['v'].append(i+1)

		for i in range(0, len(path_edges)):
			data['id'].append(i)
			data['geometry'].append(LineString(path_edges[i]))

		edges_df = pd.DataFrame(data, columns = ['id', 'u', 'v', 'geometry'])
		self._edges_gdf = geopandas.GeoDataFrame(edges_df, geometry='geometry')

		length_array = []

		for line in self._edges_gdf.geometry:
		    length_array.append(line.length)

		self._max_segment_length = max(length_array)

		return self._edges_gdf

	def _get_path_nodes(self):
		path_nodes = []
		data = {'id': [],'geometry': []}

		for i in range(0, len(self._path_points)):
			data['id'].append(i)
			data['geometry'].append(Point(self._path_points[i][0], self._path_points[i][1]))

		nodes_df = pd.DataFrame(data, columns = ['id', 'geometry'])
		self._nodes_gdf = geopandas.GeoDataFrame(nodes_df, geometry='geometry')

		self._nodes_spatial_index = self._nodes_gdf.sindex

		return self._nodes_gdf

	def _plot_path(self):
		edges = self._get_path_edges()
		nodes = self._get_path_nodes()

		edges.plot(ax=self._axis, color='b')
		nodes.plot(ax=self._axis, marker='.', color='g')

	def _plot_matching_output(self):
		for point in self._matched_points:
			plt.plot(point.x, point.y, 'k.')

	def get_longest_segment_length():
		gdf_edges = get_gdf_edges()
		length_array = []

		for line in gdf_edges.geometry:
		    length_array.append(line.length)

		return max(length_array)

if __name__ == '__main__':
    try:

		road_estimator = estimate_road()
		path_points = road_estimator._get_path_points()

		rospy.spin()

    except Exception as e:
        print(e)