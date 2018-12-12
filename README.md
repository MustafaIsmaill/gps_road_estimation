**GPS Road Estimation:**

gps_road_estimation is a **ROS** package written in **Python** that matches gps points to roads.

**Package Inputs:**

1. path way points (nav_msgs/Path.msg)
2. gps points (sensor_msgs/NavSatFix.msg)

**Package Outputs:**

1. A point for each gps point, representing the projection of the gps point on the path (geometry_msgs/PointStamped.msg) and is published on the topic (/gps_projected_points).
**Note**: every gps point and its corresponding projected point have the same time stamp.

**How to use the package:**

1. first run the /road_processing_planning package (the package runs the service /path_getter that is used by the /gps_road_estimation package)
2. run the /gps_road_estimation package through the launch command `roslaunch gps_road_estimation gps_road_estimator.launch`
3. run the bag file containing the gps points
