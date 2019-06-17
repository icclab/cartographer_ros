Notes
=============

These are some gathered notes while playing around with cartographer_ros and the TurtleBot3/TurtleBot2.

Building & Installation
-------------

If you would like to install from source on your local machine follow the *Building & Installation* instructions `here
<https://google-cartographer-ros.readthedocs.io/en/latest/>`_ (Scroll down) or `here for TurtleBot integration <https://google-cartographer-ros-for-turtlebots.readthedocs.io/en/latest/>`_.

**TIP:** Make a seperate catkin workspace for cartographer and its dependencies. 

**TIP:** Once you have successfully built and installed Cartographer, replace inside your catkin workspace *cartographer_ros* with this repository by cloning it and then rebuilding the workspace ``catkin_make_isolated --install --use-ninja``.

Another option would be to run a Docker container using one of the provided Dockerfiles found in the git repository (a github token is required to build the Docker image).

Understanding Cartographer
-------------

Read through the following documentation:

+ `Cartographer Documentation`_
+ `Cartographer ROS Documentation`_
+ **It is highly reccomended to read through ALL the docs in this directory**
+ **RUN THE DEMOS**

.. _Cartographer Documentation: https://media.readthedocs.org/pdf/google-cartographer/latest/google-cartographer.pdf
.. _Cartographer ROS Documentation: https://media.readthedocs.org/pdf/google-cartographer-ros/latest/google-cartographer-ros.pdf

Takeaways: 

+ Cartographer has two systems: local and global SLAM.
+ Local SLAM builds a locally consistent set of submaps and ties them together.
+ Global SLAM finds loop closure constraints by scan-matching scans against submaps.
+ Localization mode is useful when extending a pre-made map (pass in a pre-made map as an argument and the robot will find itself on the map and continue to build it).
+ Tuning Cartographer is a pain though there are tools to evaluate SLAM.
+ IMU data is optional for 2D SLAM (used for TB3).
+ Call the services finish_trajectory and write_state to save SLAM as a .pbstream file
+ Occupancy grid node listen to submaps published by SLAM and builds a ROS occupancy_grid out of them and publishes the /map topic (expensive and slow though necessary because TurtleBot nav stack cannot deal with Cartographer's submaps directly).
+ There are hundreds of parameters that can be set in the configuration files (.lua files). Some are described in the Tuning section mentioned above.

Testing Cartographer with TurtleBot
-------------

The minimal_exploration.launch needs to first be launched on the TurtleBot (or any other equivalent minimal.launch).

In the /cartographer_ros/launch directory there are two launch files for the TB3 and TB2. The configuration files can be found in the /cartographer_ros/configuration_files directory. 

+ *turtlebot$_slam.launch* is to build a map from scratch. Configuration file is called *turtlebot$_slam.lua*.
+ *turtlebot$_localization.launch* is to extend or localize the robot in a pre existing .pbstream map state. Simply include *load_state_filename:=$PATH_TO_MAP* as an argument when launching file. Configuration file is called *turtlebot$_localization.slam*.

Testing Cartographer with Summit_xl
-------------

**Summit in simulation:**
  1. ``roslaunch icclab_summit_xl irlab_summit_xl_cartographer.launch``
  2. ``ROS_NAMESPACE=summit_xl roslaunch cartographer_ros summit_xl_slam.launch use_sim_time:=true launch_rviz:=false``
  
**Summit ITR:**
  1. ``roslaunch icclab_summit_xl irlab_summit_xl_cartographer.launch sim:=false``
  2. ``ROS_NAMESPACE=summit_xl roslaunch cartographer_ros summit_xl_slam.launch use_sim_time:=false launch_rviz:=false``

Tuning
-------------
Make sure to read *tuning.rst* and *algo_walkthrough.rst* in the docs.

**Tips to increase stability of system if unstable:**

+ decrease speed and acceleration of robot
+ use ceres scan matcher and not online correlative scan matching (this is very expensive)
+ decrease map quality

**Tips while tuning:**

+ disable global SLAM to tune local SLAM by setting ``POSE_GRAPH.optimize_every_n_nodes = 0``
+ tune with ``TRAJECTORY_BUILDER_2D.use_imu_data = false``
+ set ``POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0`` if using odometry from wheel encoders since they often have a high uncertainty in rotation

**Odom frame:**

+ Can choose whether to let cartographer or wheel odometry to broadcast the transform "odom -> base_link"
+ Either way need to make sure only one of these components is providing the transform
+ To configure cartographer to publish the transform set ``provide_odom_frame = true`` in lua configuration file
+ `More info here`_ 

.. _More info here: https://github.com/googlecartographer/cartographer_ros/issues/1095#issuecomment-441871307 

Useful tools
-------------
**Step to save a map in a serialized state**
  1. ``rosservice call /write_state "filename: '$MAP_NAME.pbstream' include_unfinished_submaps: false"`` (**TIP:** press tab after ``/write_state`` to get arguments and note map gets saved by default to ``~/.ros``)

**Steps to convert a serialized Cartographer state (pbstream format) into a static occupancy grid.**
  * To publish map as a topic run ``rosrun cartographer_ros cartographer_pbstream_map_publisher -pbstream_filename $(filename).pbstream``
  * To convert serialized map to file formats *.pgm + *.yaml run ``rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename $(filename).pbstream``
  
**Validate sensor data.** 
  1. Record desired topics using ``rosbag record TOPIC1 [TOPIC2 TOPIC3 ...]``
  2. Validate rosbag using ``rosrun cartographer_ros cartographer_rosbag_validate -bag_filename $BAG_FILENAME.bag``

**Reccomended time deltas for consecutive messages on topics (based on output of rasbag_validate):**
  + IMU: [0.0005, 0.005] s with no jitter
  + Scan: [0.005, 0.05] s with no jitter
  
**Steps to add gravity as part of linear acceleration in imu data (if missing).**
  1. remap imu_in to the name of imu topic e.g. for TB2 add the following ``<remap from="imu_in" to="/mobile_base/sensors/imu_data" />`` as part of the flat_world_imu_node
  2. ``rosrun cartographer_turtlebot cartographer_flat_world_imu_node`` (need to have cartographer_turtlebot installed)
  3. verify ``rostopic echo imu_out``

**Add the following code to remove unwanted tf frames.**

.. code-block:: launch

  <node name="tf_remove_frames" pkg="cartographer_ros"
      type="tf_remove_frames.py">
    <remap from="tf_out" to="/tf" />
    <rosparam param="remove_frames">
      - map
      - odom_combined
    </rosparam>
  </node>


Things to Consider
-------------

+ *submaps.resolution* should be matching with resoution in the .pbstream file when doing localization. 
+ Cartographer requires huge amounts of computational resources so it is not reccomended to run Cartographer on the Turtlebot.
+ ``/use_sim_time`` should be set to true if running cartographer with simulated robot + world

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
+ Giving cartographer more computational power

**NOTE:** These errors require further investigation and continued documentation/testing on these issues is greatly appreciated.

GitHub issues to check out
--------------------
+ Splitting local and global SLAM on different machines: https://github.com/googlecartographer/cartographer_ros/issues/819
+ Odom frame transform to map frame unstable: https://github.com/googlecartographer/cartographer_ros/issues/1090
+ Triggering global localization on service request: https://github.com/googlecartographer/cartographer_ros/issues/1083
+ Using landmarks: https://github.com/googlecartographer/cartographer_ros/issues/1067
