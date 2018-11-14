#!/usr/bin/env python

import rospy
import osmnx as ox

from road_info_osm_extract.msg import road
from road_info_osm_extract.msg import road_lists

class extract_road_info:

	def __init__(self, node_name, place_name, publish_rate):
		
		#initialise class variables
		self._place_name = place_name
		self._ros_node_name = node_name
		self._list_of_roads = road_lists()
		self._publish_rate = publish_rate

		#create publisher
		self._road_info_publisher = rospy.Publisher(self._ros_node_name, road_lists, queue_size=100)

		#create a node
		rospy.init_node('extract_road_info', anonymous=False)
		self._publish_rate = rospy.Rate(self._publish_rate) # 1hz

		#convert place in a map to a graph
		self._graph = ox.graph_from_place(self._place_name, network_type='drive')
		self._graph_proj = ox.project_graph(self._graph)

		#extract edges and nodes
		#edges define the geometry of the road
		#nodes define the start and end points of each road
		self._nodes, self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=True, edges=True)

	def _publish_road_info(self):

		self._parse_road_info()

		while not rospy.is_shutdown():
		    
		    self._road_info_publisher.publish(self._list_of_roads)
		    print("list of roads published")
		    self._publish_rate.sleep()

	def _get_start_x(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].x[start_node_id]

	def _get_start_y(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].y[start_node_id]

	def _get_end_x(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].x[start_node_id]

	def _get_end_y(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].y[start_node_id]

	def _get_start_lat(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].lat[start_node_id]

	def _get_start_lon(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].lon[start_node_id]

	def _get_end_lat(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].lat[start_node_id]

	def _get_end_lon(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].lon[start_node_id]

	def _get_edge_length(self, edge_id):

		return self._edges[:].length[edge_id]

	def _get_edge_direction(self, edge_id):

		direction = self._edges[:].oneway[edge_id]
		direction = float(direction)
		return direction

	def _parse_road_info(self):

		for i in range(0,len(self._edges)):
			
			edge = road()

			edge.info.insert(0, i)	#road ID
			edge.info.insert(1, self._get_start_x(i))
			edge.info.insert(2, self._get_start_y(i))
			edge.info.insert(3, self._get_start_lat(i))
			edge.info.insert(4, self._get_start_lon(i))
			edge.info.insert(5, self._get_end_x(i))
			edge.info.insert(6, self._get_end_y(i))
			edge.info.insert(7, self._get_end_lat(i))
			edge.info.insert(8, self._get_end_lon(i))
			edge.info.insert(9, self._get_edge_length(i))
			edge.info.insert(10, self._get_edge_direction(i))

			self._list_of_roads.roads.insert(i, edge)
