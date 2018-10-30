Notes
=============

These are some gathered notes while playing around with cartographer_ros and the TurtleBot3/TurtleBot2.

Building & Installation
-------------

If you would like to install from source on your local machine follow the *Building & Installation* instructions [here](https://google-cartographer-ros.readthedocs.io/en/latest/) (Scroll down).

**TIP:** Make a seperate catkin workspace for cartographer and its dependencies. 

Another option would be to run a Docker container using one of the provided Dockerfiles found in the git repository (a github token is required to build the Docker image).

Understanding Cartographer
-------------

Read through the following documentation:

+ [Configuration](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)
+ [Tuning](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)
+ [ROS API](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html)
+ [Cartographer Documentation](https://media.readthedocs.org/pdf/google-cartographer/latest/google-cartographer.pdf)

Takeaways: 

+ Cartographer has two systems: local and global SLAM.
+ Local SLAM builds a locally consistent set of submaps and ties them together.
+ Global SLAM finds loop closure constraints by scan-matching scans against submaps.
+ Localization mode is useful when extending a pre-made map (pass in a pre-made map as an argument and the robot will find itself on the map and continue to build it).
+ Tuning Cartographer is a pain though there are tools to evaluate SLAM.
+ IMU data is optional for 2D SLAM (used for TB3).
+ Call the services finish_trajectory and write_state to save SLAM as a .pbstream file.
+ Occupancy grid node listen to submaps published by SLAM and builds a ROS occupancy_grid out of them and publishes the /map topic (expensive and slow though necessary because TurtleBot nav stack cannot deal with Cartographer's submaps directly).
+ There are hundreds of parameters that can be set in the configuration files (.lua files). Some are described in the Tuning section mentioned above.

Testing Cartographer with TurtleBot
-------------

The minimal_exploration.launch needs to first be launched on the TurtleBot (or any other equivalent minimal.launch).

In the /cartographer_ros/launch directory there are two launch files for the TB3 and TB2. The configuration files can be found in the /cartographer_ros/configuration_files directory. 

+ *turtlebot$_slam.launch* is to build a map from scratch. Configuration file is called *turtlebot$_slam.lua*.
+ *turtlebot$_localization.launch* is to extend or localize the robot in a pre existing .pbstream map state. Simply include *load_state_filename:=$PATH_TO_MAP* as an argument when launching file. Configuration file is called *turtlebot$_localization.slam*.

Useful tools
-------------

Steps to convert a serialized Cartographer state (pbstream format) into a static occupancy grid. The following steps will output a .yaml and .pgm mapfile.
  1. rosrun cartographer_ros cartographer_pbstream_map_publisher -pbstream_filename $(filename).pbstream
  2. rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename $(filename).pbstream


Things to Consider
-------------

+ *submaps.resolution* should be matching with resoution in the .pbstream file when doing localization. 
+ Cartographer requires huge amounts of computational resources so it is not reccomended to run Cartographer on the Turtlebot.
+ Current configuration files are not very fine tuned (mostly the same as default settings).

Common Errors/Warnings
-------------

These are common errors that cause the cartographer node to fail or not work properly. 

Error from robot: 

*W0913 14:14:41.000000 21962 tf_bridge.cc:52] Lookup would require extrapolation into the future.  Requested time 1536840881.193535400 but the latest data is at time 1536840881.046552729, when looking up transform from frame [odom] to frame [imu_link]*

Error from cartographer:

*F0913 14:16:21.000000 21962 pose_extrapolator.cc:229] Check failed: time >= imu_tracker->time() (636724377780390007 vs. 636724377790000007)*

Things that seem to help:

+ Lowering latency 
+ Not using IMU data
+ Giving cartographer more resources

**NOTE:** These errors require further investigation and continued documentation/testing on these issues is greatly appreciated.

