##**Road Info Extraction:**

road_info_osm_extract is a **ROS** package written in **Python** that extracts road information from a specified area in open street maps.

##**How to use the package:**

1- go to [Nominatim Open Street Maps application] (https://nominatim.openstreetmap.org/) and search the area that you want to extract information from, the application should highlight to you the exact area that will be downloaded according to your **search string**.  
2- copy your search string and go to the file "gps_road_estimation/road_info_osm_extract/scripts/extract_road_info.py" and assign the "place_name" variable to your search string.  
3- in the file "gps_road_estimation/road_info_osm_extract/scripts/extract_road_info.py", rename the "node_name" and change the "publish_rate" variables to match your needs (optional).  
4- from the terminal run the following command `$ roslaunch road_info_osm_extract road_info_osm_extract.launch`  
5- subscribe to the "/road_info" topic to receive the road information

##**Package Inputs**

1- name of place or area in a map  
2- name of ros node  
3- rate of publishing the road information in Hertz  

##**Package Outputs**

- the output of the package is the custom message "road_lists" that is published on the topic "/road_info"  
- the message "road_lists" is a list of roads (a list of lists), each list (road) is an array with 11 elements.  
- the list elements are a definition of the road. each road is defined by a start node and an end node. each node is defined by 4 parameters, x-coordinate, y-coordinate, latitude and longitude, all with respect to the open street map coordinates.  
- the road list elements are arranged in the following order:  

0- road id  
1- the x-coordinate of the start node  
2- the y-coordinate of the start node  
3- the latitude of the start node  
4- the longitude of the start node  
5- the x-coordinate of the end node  
6- the y-coordinate of the end node  
7- the latitude of the end node  
8- the longitude of the end node  
9- the length of the road  
10- a boolean that represents if the road is a one way road or not (0 = not a one way road, 1 = a one way road)  

##
