#!/usr/bin/env python

# place_name = 'leganes tecnologico (fase 1) ,leganes, madrid,spain'

import rospy

from map_extract import *

if __name__ == '__main__':

	try:

		#package inputs
		#edit these 3 variables to match your criterea
		place_name = 'leganes tecnologico (fase 1) ,leganes, madrid,spain'
		node_name = 'road_info_extracter'
		publish_rate = 0.1 # 0.1Hz

		#create a map object
		map = map_extract(node_name, place_name, publish_rate)

		#publish road information
		map._publish_road_info()

	except rospy.ROSInterruptException:
		print("exception occured")
