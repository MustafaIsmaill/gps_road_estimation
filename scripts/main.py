#!/usr/bin/env python

# place_name = 'leganes tecnologico (fase 1) ,leganes, madrid,spain'

import rospy

from extract_road_info import *

if __name__ == '__main__':

	try:

		place_name = 'leganes tecnologico (fase 1) ,leganes, madrid,spain'
		node_name = 'road_info_extracter'
		publish_rate = 0.1 # 0.1Hz

		map = extract_road_info(node_name, place_name, publish_rate)

		map._publish_road_info()

	except rospy.ROSInterruptException:
		print("exception occured")
