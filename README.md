**GPS Road Estimation:**

gps_road_estimation is a **ROS** package written in **Python** that matches gps points to open street maps' roads.

**Subscribed Topics:**

/ada/fix (sensor_msgs/NavSatFix.msg)

**Published Topics:**

/gps_projected_points (geometry_msgs/PointStamped.msg)

**How to use the package:**

1. change the name of the subscribed topic from the file `/gps_road_estimation/params/map_match.yaml`, default is `ada/fix`
2. run the /road_processing_planning package using the launch command `roslaunch road_processing_planning route_points.launch`
3. run the /gps_road_estimation package using the launch command `roslaunch gps_road_estimation gps_road_estimator.launch`

**ROS Dependencies:**

1. geometry_msgs
2. sensor_msgs
3. nav_msgs
4. /road_processing_planning (package)

**Python Dependencies:**

1. utm (pip2 install utm)
2. geopandas (pip2 install geopandas)
3. pandas (pip2 install pandas)
4. shapely (pip2 install shapely)
