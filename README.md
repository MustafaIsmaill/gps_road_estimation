**GPS Road Estimation:**

gps_road_estimation is a **ROS** package written in **Python** that matches gps points to roads.

**Package Inputs:**

1. path way points  
2. gps points  

**Package Outputs:**

1. a corresponding or a projected point for each gps point on the path published on the topic /gps_projected_points. each published point has the same time stamp as the gps point corresponding to it.

**How to use the package:**

1. first run the /road_processing_planning package (the package runs the service /path_getter that is used by the /gps_road_estimation package)
2. run the /gps_road_estimation package through the launch command `roslaunch gps_road_estimation gps_road_estimator.launch`
3. run the bag file containing the gps points
