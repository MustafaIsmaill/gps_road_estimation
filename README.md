# GPS Road Estimation:

gps_road_estimation is a **ROS** package written in **Python** that matches vehicle/robot position to open street maps' roads. The package can work with two different types of inputs, NavSatFix and Odometry messages.

## 1. Working with NavSatFix messages

run using `$ roslaunch gps_road_estimation fix_road_estimator.launch`

### 1.1 Subscribed Topics:

/ada/fix (sensor_msgs/NavSatFix.msg) is the default topic name, if you want to change it go to the file `/gps_road_estimation/launch/fix_road_estimator.launch` and change the "sub_topic" argument value.

## 2. Working with Odometry messages

run using `$ roslaunch gps_road_estimation odom_road_estimator.launch`

### 2.1 Subscribed Topics:

/ada/odometry_fusion_ekf (nav_msgs/Odometry.msg) is the default topic name, if you want to change it go to the file `/gps_road_estimation/params/road_estimation.yaml` and change the "pub_topic" parameter value.

## 3 Published Topics:

1. /ada/projected_odometry (nav_msgs/Odometry.msg). 
2. /ada/goal_status (actionlib_msgs/GoalStatusArray.msg). The goal status array msg has a goal for each way point in the path msg (excluding the first point which is the vehicle's start position), and the status of each goal indicates if the goal has been reached or not.

## ROS Dependencies:

1. /gps_umd (package) install using `$ sudo apt-get install ros-kinetic-gps-umd`
2. /road_processing_planning (package) run using `$ roslaunch road_processing_planning route_points.launch`

## Python Dependencies:

1. utm install `$ sudo pip2 install utm`
2. geopandas install `$ sudo pip2 install geopandas`
3. pandas install `$ sudo pip2 install pandas`
4. shapely install `$ sudo pip2 install shapely`

