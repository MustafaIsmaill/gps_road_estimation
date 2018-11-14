#!/usr/bin/env python

import rospy
import osmnx as ox

from road_info_osm_extract.msg import road
from road_info_osm_extract.msg import road_lists

class map_extract:

	def __init__(self, node_name, place_name, publish_rate):
		
		#initialise class variables
		self._place_name = place_name
		self._ros_node_name = node_name
		self._list_of_roads = road_lists()
		self._publish_rate = publish_rate

		#create publisher
		self._road_info_publisher = rospy.Publisher("road_info", road_lists, queue_size=100)

		#create a node
		rospy.init_node(self._ros_node_name, anonymous=False)
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

	#returns the x-coordinate of the start node
	def _get_start_x(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].x[start_node_id]

	#returns the y-coordinate of the start node
	def _get_start_y(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].y[start_node_id]

	#returns the x-coordinate of the end node
	def _get_end_x(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].x[start_node_id]

	#returns the y-coordinate of the end node
	def _get_end_y(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].y[start_node_id]

	#returns the latitude of the start node
	def _get_start_lat(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].lat[start_node_id]

	#returns the longitude of the start node
	def _get_start_lon(self, edge_id):

		start_node_id = self._edges[:].u[edge_id]
		return self._nodes[:].lon[start_node_id]

	#returns the latitude of the end node
	def _get_end_lat(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].lat[start_node_id]

	#returns the longitude of the end node
	def _get_end_lon(self, edge_id):

		start_node_id = self._edges[:].v[edge_id]
		return self._nodes[:].lon[start_node_id]

	#returns the length of the edge or road
	def _get_edge_length(self, edge_id):

		return self._edges[:].length[edge_id]

	#returns a boolean indicating whether a road is a one way or not
	def _get_edge_direction(self, edge_id):

		direction = self._edges[:].oneway[edge_id]
		direction = float(direction)
		return direction

	#parses the edges and nodes into readable lists
	def _parse_road_info(self):

		#loop through all roads or "edges"
		for i in range(0,len(self._edges)):
			
			#create a msg of the type "road"
			edge = road()

			#fill the road info into the "road" message
			edge.info.insert(0, i)	#road ID
			edge.info.insert(1, self._get_start_x(i))	#x-coordinate of start node
			edge.info.insert(2, self._get_start_y(i))	#y-coordinate of start node
			edge.info.insert(3, self._get_start_lat(i))	#latitude of start node
			edge.info.insert(4, self._get_start_lon(i))	#longitude of start node
			edge.info.insert(5, self._get_end_x(i))		#x-coordinate of end node
			edge.info.insert(6, self._get_end_y(i))		#y-coordinate of end node
			edge.info.insert(7, self._get_end_lat(i))	#latitude of end node
			edge.info.insert(8, self._get_end_lon(i))	#longitude of end node
			edge.info.insert(9, self._get_edge_length(i))	#length of road in meters
			edge.info.insert(10, self._get_edge_direction(i))	#one way road or not

			#add this road to the list of roads
			self._list_of_roads.roads.insert(i, edge)
