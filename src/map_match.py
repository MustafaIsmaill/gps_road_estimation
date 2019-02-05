import rospy
from collections import OrderedDict

import time

class map_match:
	def __init__(self, path):
		self.path = path
		self.nodes_gdf = self.path.get_path_nodes()
		self.edges_gdf = self.path.get_path_edges()
		self.nodes_spatial_index = self.path.get_nodes_spatial_idx()

		# self.search_radius = 1.5

		# self.file = open("/home/mustafaismail/Documents/GP/catkin_ws/src/gps_road_estimation/src/error_data/gps_no_shift.txt", "w") 

	def get_projected_p(self, odom_point):
		self.search_radius = 1.0

		t = time.time()
		distance = []
		for edge in self.get_candidate_edges(odom_point):
			line_string = self.edges_gdf[(self.edges_gdf.u == edge[0]) & (self.edges_gdf.v == edge[1])].geometry
			distance.append([line_string.distance(odom_point), edge, line_string])

		tup = min(distance, key = lambda t: t[0].values)
		true_edge = tup[1]
		true_edge_geom = tup[2].item()
		
		# self.file.write(str(tup[0].values))
		# rospy.loginfo(tup[0].values)

		projected_point = true_edge_geom.interpolate(true_edge_geom.project(odom_point)) # projected point

		# rospy.loginfo(time.time() - t)
		# self.search_radius = self.path.get_edge_length(true_edge[0])
		return projected_point, true_edge[0]

	def get_candidate_edges(self, odom_point):
		candidate_edges = []
		for node_id in self.get_candidate_nodes(odom_point):
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

	def get_candidate_nodes(self, odom_point):
		while True:
			circle = odom_point.buffer(self.search_radius) 
			possible_matches_index = list(self.nodes_spatial_index.intersection((circle.bounds))) 
			possible_matches = self.nodes_gdf.iloc[possible_matches_index] 
			precise_matches = possible_matches[possible_matches.intersects(circle)]
			candidate_nodes = list(precise_matches.index)

			if len(candidate_nodes) != 0:
				break
			else:
				# rospy.loginfo(self.search_radius)
				self.search_radius += 5

		return candidate_nodes
