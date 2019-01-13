import rospy

import math
import make_path as mp

from collections import OrderedDict
from shapely.geometry import Point

class map_match:
	def __init__(self, path_points):
		self.path_points = path_points

		self.nodes_gdf = mp.get_path_nodes(path_points=self.path_points)
		self.edges_gdf = mp.get_path_edges(path_points=self.path_points)
		self.nodes_spatial_index = mp.get_nodes_spatial_idx(nodes_gdf=self.nodes_gdf)
		self.max_segment_length = mp.get_longest_segment_len(edges_gdf=self.edges_gdf)

		self.prev_dist = 0.0
		self.ignorance_bool = 0

	def get_pp_edge(self, gps_point):
		distance = []
		length = []
		for edge in self.get_candidate_edges(gps_point):
			line_string = self.edges_gdf[(self.edges_gdf.u == edge[0]) & (self.edges_gdf.v == edge[1])].geometry
			distance.append([line_string.distance(gps_point), edge, line_string])
			dis = line_string.distance(gps_point)
			length.append(dis.iloc[0])

		_, idx = min((length[i],i) for i in xrange(len(length)))
		true_edge = distance[idx][1]
		true_edge_geom = distance[idx][2].item()
		projected_point = true_edge_geom.interpolate(true_edge_geom.project(gps_point)) # projected point

		return projected_point, true_edge[1]

	def get_candidate_edges(self, gps_point):
		candidate_edges = []
		for node_id in self.get_candidate_nodes(gps_point):
			# first node
			if node_id == 0:
				point_tuple_out = (node_id, node_id+1)
				candidate_edges.append(point_tuple_out)
			# last node
			elif node_id == len(self.nodes_gdf)-1:
			    point_tuple_in = (node_id-1, node_id)
			    candidate_edges.append(point_tuple_in)
			# any other node
			else:
			    point_tuple_in = (node_id-1, node_id)
			    point_tuple_out = (node_id, node_id+1)
			    candidate_edges.append(point_tuple_in)
			    candidate_edges.append(point_tuple_out)

		candidate_edges = list(OrderedDict.fromkeys(candidate_edges))
		return candidate_edges

	def get_candidate_nodes(self, gps_point):
		while True:
			circle = gps_point.buffer(self.max_segment_length) 
			possible_matches_index = list(self.nodes_spatial_index.intersection((circle.bounds))) 
			possible_matches = self.nodes_gdf.iloc[possible_matches_index] 
			precise_matches = possible_matches[possible_matches.intersects(circle)]
			candidate_nodes = list(precise_matches.index)

			if len(candidate_nodes) != 0:
				break
			else:
				rospy.loginfo("cannot find any near nodes, increasing search circle")
				self.max_segment_length += 100 

		candidate_nodes.sort()
		return candidate_nodes

	def update_goal_status(self, gps_point, matched_edge, goal_arr):
		next_wp = Point(self.path_points[matched_edge][0], self.path_points[matched_edge][1])
		self.curr_dist = self.get_dist(gps_point, next_wp)

		goal_arr.header.stamp = rospy.Time.now()

		if matched_edge == 1:
			goal_arr.status_list[matched_edge-1].status = 1
			goal_arr.status_list[matched_edge-1].text = 'ACTIVE'
		
		elif matched_edge != len(self.path_points)-1:
			goal_arr.status_list[matched_edge-1].status = 1
			goal_arr.status_list[matched_edge-1].text = 'ACTIVE'
			goal_arr.status_list[matched_edge-2].status = 3
			goal_arr.status_list[matched_edge-2].text = 'SUCCEEDED'
		
		elif matched_edge == len(self.path_points)-1:
			if self.ignorance_bool == 0:
				self.ignorance_bool = 1
			else:
				if self.curr_dist > self.prev_dist:
					goal_arr.status_list[matched_edge-1].status = 3
					goal_arr.status_list[matched_edge-1].text = 'SUCCEEDED'
				else:
					goal_arr.status_list[matched_edge-1].status = 1
					goal_arr.status_list[matched_edge-1].text = 'ACTIVE'
					goal_arr.status_list[matched_edge-2].status = 3
					goal_arr.status_list[matched_edge-2].text = 'SUCCEEDED'

		self.prev_dist = self.curr_dist
		return goal_arr

	def get_dist(self, point1, point2):
		return point1.distance(point2)

# 5.29731127447