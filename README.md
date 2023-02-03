# Masterprojekt-Intelligente-Systeme
This branch is code that is meant to work on the real robot(locobot_base).

Current State of Implementation in real robot:
* DWA+LiDAR tested and works together 
-> in the "real_robot_utku branch" branch, the commits: 88d8813c5e9a9d7070f2bea0e4a00c5819c3603f (reafactored code) and 23ab692eaab9d75da933180a4bc64e4560d66065 correspond to these working codes
* Robot Perception+Prediction Model works together
-> the final commit of branch "robot_perception_and_prediction" corresponds to this
* The current commit of the real_robot_utku branch is the code that combines everything together(DWA+LiDAR+Perception+Prediction), however this code has not been tested yet.

/// Below is a work-in progress ///

Usage of DWA+LiDAR:
* copy the simple_dwa_lidar into catkin workspace
* run catkin_make in workspace
* run from locobot: roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_base use_base:=true use_lidar:=true use_camera:=true
* run from remote pc: roslaunch simple_dwa_lidar demo.launch --goal 5.5 0.0


Usage of Robot Perception+Prediction:
* copy the simple_dwa_lidar into catkin workspace
* run catkin_make in workspace
* run: pip install -r requirements.txt
* run from locobot: roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_base use_base:=true use_lidar:=true use_camera:=true
* run from zedbox: roslaunch praxisproject_ws22 robot_perception.launch
* run from remote pc: roslaunch test_robot_perception demo.launch
