#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import osmnx as ox

from nav_msgs.msg import Odometry

ekf_out = []
gps_out = []

def plot_output():
    
    print("getting graph for plot")
    
    graph = ox.graph_from_place('leganes spain', network_type='drive')
    graph_proj = ox.project_graph(graph)
    nodes, edges = ox.graph_to_gdfs(graph_proj, nodes=True, edges=True)
    
    _, ax = plt.subplots()
    edges.plot(ax=ax)
    print("done plotting open street map")
    
    for i in range(0, len(ekf_out)):
        x = ekf_out[i][0]
        y = ekf_out[i][1]

        rospy.loginfo("Plotting ekf output")
        plt.plot(x, y, 'r.', alpha=1)

    for i in range(0, len(gps_out)):
        x = gps_out[i][0]
        y = gps_out[i][1]

        rospy.loginfo("Plotting gps odometry")
        plt.plot(x, y, 'k.', alpha=0.1)

    plt.show()

def ekf_callback(ekf_msg):
    ekf_utmx = ekf_msg.pose.pose.position.x
    ekf_utmy = ekf_msg.pose.pose.position.y

    ekf_out.append((ekf_utmx, ekf_utmy))

    if ekf_msg.header.seq >= 1658/6: # 1658
        ekf_sub.unregister()
        plot_output()

def gps_callback(gps_msg):
    gps_utmx = gps_msg.pose.pose.position.x
    gps_utmy = gps_msg.pose.pose.position.y
    
    gps_out.append((gps_utmx, gps_utmy))

    if gps_msg.header.seq >= 3137/6: # 3137
        gps_sub.unregister()    
    
if __name__ == '__main__':
    try:

        rospy.init_node('ekf_output_plot', anonymous=False)
        ekf_sub = rospy.Subscriber("/ada/odometry_fusion_ekf", Odometry, ekf_callback)
        gps_sub = rospy.Subscriber("/ada/gps_odometry_corrected", Odometry, gps_callback)
        rospy.loginfo("ready to subscribe")
        
        rospy.spin()

    except Exception as e:
        print(e)

# seq: 16382
# /ada/gps_odometry