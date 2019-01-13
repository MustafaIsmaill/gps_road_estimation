#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import osmnx as ox

from nav_msgs.msg import Odometry

pp_out = []
gps_out = []

def plot_edges():
    print("getting graph for plot")
    edges = ox.graph_to_gdfs(ox.project_graph(ox.graph_from_place('leganes spain', network_type='drive')), nodes=False, edges=True)
    _, ax = plt.subplots()
    edges.plot(ax=ax)
    print("done plotting open street map")
    
def plot_output():
    for i in range(0, len(pp_out)):
        rospy.loginfo("plotting projected points")
        plt.plot(pp_out[i][0], pp_out[i][1], 'r.', alpha=1)

    for i in range(0, len(gps_out)):
        rospy.loginfo("plotting gps odometry")
        plt.plot(gps_out[i][0], gps_out[i][1], 'k.', alpha=1)

    plt.show()

def gps_pp_cb(pp):
    x = pp.pose.pose.position.x
    y = pp.pose.pose.position.y

    pp_out.append((x, y))

    if pp.header.seq == 1:
        plot_edges()

    if pp.header.seq >= 7653: # 1375
        gps_pp_sub.unregister()
        plot_output()

def gps_cb(gps):
    x = gps.pose.pose.position.x
    y = gps.pose.pose.position.y
    
    gps_out.append((x, y))

    if gps.header.seq >= 7653: #2700 fix 
        gps_sub.unregister()    
    
if __name__ == '__main__':
    try:

        rospy.init_node('mm_output_plot', anonymous=False)
        gps_sub = rospy.Subscriber("/ada/odometry_fusion_ekf", Odometry, gps_cb)
        gps_pp_sub = rospy.Subscriber("/ada/projected_points_odometry", Odometry, gps_pp_cb)
        
        rospy.spin()

    except Exception as e:
        print(e)
