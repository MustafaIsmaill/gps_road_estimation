# GPS Road Estimation:

gps_road_estimation is a **ROS** package written in **Python** that matches vehicle/robot position to open street maps' roads. The package can work in two modes, the first mode works directly with NavSatFix messages and the second mode works directly with Odometry messages.

## 1. Working with NavSatFix messages

run using `$ roslaunch gps_road_estimation fix_road_estimator.launch`

### 1.1 Subscribed Topics:

/ada/fix (sensor_msgs/NavSatFix.msg). Note: if you want to change the name of this topic you can go to the file `/gps_road_estimation/launch/fix_road_estimator.launch` and change the "sub_topic" argument value.

### 1.2 Published Topics:

/ada/gps_pp_odometry (nav_msgs/Odometry.msg). Note: if you want to change the name of this topic you can go to the file `/gps_road_estimation/params/fix_estimation.yaml` and change the "pp_pub_topic" parameter value.

## 2. Working with Odometry messages

run using `$ roslaunch gps_road_estimation odom_road_estimator.launch`

### 2.1 Subscribed Topics:

/ada/odometry_fusion_ekf (nav_msgs/Odometry.msg). Note: if you want to change the name of this topic you can go to the file `/gps_road_estimation/params/odom_estimation.yaml` and change the "pub_topic" parameter value.

### 2.2 Published Topics:

/ada/odometry_fusion_projected (nav_msgs/Odometry.msg). Note: if you want to change the name of this topic you can go to the file `/gps_road_estimation/params/fix_estimation.yaml` and change the "pp_pub_topic" parameter value.

## ROS Dependencies:

1. /gps_umd (package) install using `$ sudo apt-get install ros-kinetic-gps-umd`
2. /road_processing_planning (package) run using `$ roslaunch road_processing_planning route_points.launch`

## Python Dependencies:

1. utm install `$ sudo pip2 install utm`
2. geopandas install `$ sudo pip2 install geopandas`
3. pandas install `$ sudo pip2 install pandas`
4. shapely install `$ sudo pip2 install shapely`
