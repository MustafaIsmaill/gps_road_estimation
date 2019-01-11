import geopandas
import pandas as pd

from shapely.geometry import Point
from shapely.geometry import LineString

def get_path_edges(path_points):
	path_edges = []
	data = {'id': [], 'u': [], 'v': [], 'geometry': []}

	for i in range(0, len(path_points)-1):
		path_edges.append([(path_points[i][0], path_points[i][1]), (path_points[i+1][0], path_points[i+1][1])])

	for i in range(0, len(path_edges)):
		data['u'].append(i)
		data['v'].append(i+1)

	for i in range(0, len(path_edges)):
		data['id'].append(i)
		data['geometry'].append(LineString(path_edges[i]))

	edges_df = pd.DataFrame(data, columns = ['id', 'u', 'v', 'geometry'])
	return geopandas.GeoDataFrame(edges_df, geometry='geometry')

def get_path_nodes(path_points):
	path_nodes = []
	data = {'id': [],'geometry': []}

	for i in range(0, len(path_points)):
		data['id'].append(i)
		data['geometry'].append(Point(path_points[i][0], path_points[i][1]))

	nodes_df = pd.DataFrame(data, columns = ['id', 'geometry'])
	return geopandas.GeoDataFrame(nodes_df, geometry='geometry')

def get_nodes_spatial_idx(nodes_gdf):
	return nodes_gdf.sindex

def get_longest_segment_len(edges_gdf):
	length_array = []
	for line in edges_gdf.geometry:
	    length_array.append(line.length)

	return max(length_array)