**GPS Road Estimation:**

gps_road_estimation is a **ROS** package written in **Python** that matches gps points to roads.

**Subscribed Topics:**

1. gps raw data //
/ada/fix (sensor_msgs/NavSatFix.msg)

**Published Topics:**

1. matched gps points //
/gps_projected_points (geometry_msgs/PointStamped.msg)

**How to use the package:**

1. first run the /road_processing_planning package
2. run the /gps_road_estimation package through the launch command `$roslaunch gps_road_estimation gps_road_estimator.launch`
3. run the bag file containing the gps points

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
