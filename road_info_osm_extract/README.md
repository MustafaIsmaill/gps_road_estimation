##**Road Info Extraction:**

road_info_osm_extract is a ros package written in python that extracts road information from a specified area in open street maps.

##**Process:**

1- go to [Nominatim Open Street Maps application] (https://nominatim.openstreetmap.org/) and search the area that you want to extract information from, the application should highlight to you the exact area that will be downloaded according to your **search string**.
2- copy your search string and go to the file "gps_road_estimation/road_info_osm_extract/scripts/main.py" and assign the "place_name" variable to your search string.
3- in the file "gps_road_estimation/road_info_osm_extract/scripts/main.py", rename the "node_name" and change the "publish_rate" variables to match your needs (optional).
4- from the terminal run the following command "$ roslaunch road_info_osm_extract road_info_osm_extract.launch"

##**Package Inputs**

1- name of place or area in a map
2- name of ros node
3- rate of publishing the road information in Hertz

##**Package Outputs**

- the output of the package is the custom message "road_lists" that is published on the topic ""
