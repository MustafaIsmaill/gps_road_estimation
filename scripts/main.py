#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import pandas as pd
import geopandas
import time
import math

from shapely.geometry import Point
from shapely.geometry import LineString

def parse_path_file():
    with open(path_file) as f:
        path_list = f.readlines()
    path_list = [x.strip() for x in path_list]
    
    for point in path_list:
        x, y = point.split(',')
        point_tuple = (float(x), float(y))
        path_points.append(point_tuple)

def parse_gps_file():
    with open(gps_file) as f:
        gps_list = f.readlines()
    gps_list = [x.strip() for x in gps_list]
    
    for point in gps_list:
        x, y = point.split(',')
        point_tuple = (float(x), float(y))
        gps_points.append(point_tuple)

def plot_path_map():
    edges = get_gdf_edges()
    nodes = get_gdf_nodes()
    gps_points = get_gps_trajectory()
    
    matched_points = map_match(gps_points, nodes, edges)

    _, ax = plt.subplots()

    # for point in matched_points:
    #     plt.plot(point.x, point.y, 'y.')

    gps_points.plot(ax=ax, marker='.', color='r')
    edges.plot(ax=ax, color='b')
    nodes.plot(ax=ax, marker='*', color='k')

    plt.show()

def get_gdf_edges():
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
    edges_gdf = geopandas.GeoDataFrame(edges_df, geometry='geometry')

    return edges_gdf

def get_gdf_nodes():
    path_nodes = []
    data = {'id': [],'geometry': []}

    for i in range(0, len(path_points)):
        data['id'].append(i)
        data['geometry'].append(Point(path_points[i][0], path_points[i][1]))

    nodes_df = pd.DataFrame(data, columns = ['id', 'geometry'])
    nodes_gdf = geopandas.GeoDataFrame(nodes_df, geometry='geometry')

    return nodes_gdf

def get_gps_trajectory():
    data = {'geometry': []}

    for i in range(0, len(gps_points)):
        data['geometry'].append(Point(gps_points[i][0], gps_points[i][1]))    

    gps_df = pd.DataFrame(data, columns = ['geometry'])
    gps_gdf = geopandas.GeoDataFrame(gps_df, geometry='geometry')

    return gps_gdf

def get_longest_segment_length():
    gdf_edges = get_gdf_edges()
    length_array = []

    for line in gdf_edges.geometry:
        length_array.append(line.length)

    return max(length_array)

def map_match(gps_trajectory, gdf_nodes, gdf_edges):
    nodes_spatial_index = gdf_nodes.sindex

    for gps_point in gps_trajectory.itertuples():
        start = time.time()
        
        point = gps_point[1]

        circle = point.buffer(get_longest_segment_length()/2.0)
        possible_matches_index = list(nodes_spatial_index.intersection((circle.bounds)))
        possible_matches = gdf_nodes.iloc[possible_matches_index]
        precise_matches = possible_matches[possible_matches.intersects(circle)]
        candidate_nodes = list(precise_matches.index)

        candidate_edges = []
        for node_id in candidate_nodes:
            if node_id == len(gdf_nodes)-1:
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
            line_string = gdf_edges[(gdf_edges.u == edge[0]) & (gdf_edges.v == edge[1])].geometry
            distance.append([line_string.distance(point), edge, line_string])

            d = line_string.distance(point)
            length.append(d.iloc[0])

        _, idx = min((length[i],i) for i in xrange(len(length)))
        true_edge = distance[idx][1]
        true_edge_geom = distance[idx][2].item()
        projected_point = true_edge_geom.interpolate(true_edge_geom.project(point)) # projected point

        next_wp = true_edge_geom.coords[:][len(true_edge_geom.coords[:])-1] # next way point

        distance_to_next_wp = projected_point.distance(Point(next_wp[0], next_wp[1]))

        print("Distance to next waypoint is %f" %distance_to_next_wp)
        # print("Time taken = %f" % (time.time() - start) )

        result.append(projected_point)

    return result

def run_node():
    rospy.init_node('gps_road_estimator', anonymous=True)

    parse_path_file()
    parse_gps_file()

    plot_path_map()
    # rospy.spin()

if __name__ == '__main__':
    try:
        path_file = '/home/mustafaismail/Documents/GP/catkin_ws/src/gps_road_estimation/data/path_data.txt'
        gps_file = '/home/mustafaismail/Documents/GP/catkin_ws/src/gps_road_estimation/data/gps_data.txt'
        
        path_points = []
        gps_points = []
        result = []
    
        run_node()

    except Exception as e:
        print(e)
