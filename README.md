# Masterprojekt-Intelligente-Systeme
This branch is essentially contains two main versions of the: DWA+LiDAR and DWA+LiDAR+Perception+Prediction. 
* The DWA+LiDAR code versioning starts from the beginning of the branch and ends with the commit with the following id and full SHA erspectovely: b58d5dc and b58d5dc182360a823c66f6daf3133a6a18d47754

## Real Robot Package Structures w.r.t. Repo Branches
The packages on real robot exist in threefold:
### DWA+LiDAR
* Name of the ros package: simple_dwa_lidar
* This version of the code has a straightforward DWA implementation as a local planner and uses LiDAR data for collision avoidance. The best proposed actions from the DWA are sent to the locobot as Twist messages. As localization, locobot's own localization is used.
* This package has been tested and can be used for navigation of the robot (remark: setting the most suitable weighting parameters for cost functions, LiDAR range as well as max. linear velocity is key for.
* This package's versioning starts from the beginning of the current branch and ends with the commit with the following id and full SHA erspectovely: b58d5dc and b58d5dc182360a823c66f6daf3133a6a18d47754. 
* The current most stable version of the code is ....
### Perception+Prediction
* Name of the ros package: test_robot_perception
* This version of the code simply demonstrates how the robot perception package on zedBox and the pre-trained Auto-Regressive model establish a .
* This package's implementation as well as its detail on usage can be found on the 'robot_perception_and_prediction_model' branch of this repository.
### DWA+LiDAR+Perception+Prediction
* Name of the ros package: dwa_w_prediction
* This package makes use of the perception package and the prediction model together with the LiDAR sensor data. It then sends all of that into the local planner DWA for trajectory calculation and decision making. 
* Currently this package is not working as intended. The flow of data has been checked multiple times and all the different components seem to be working, however during testing with the real robot, it was observed that there is a huge delay in the Twist messages being sent (i.e. robot reacts too slow to the environment). One component of the delay is the headless MarkerArray's that cannot be synchronzied with other messages. However, based on tests, there seems to be even more reason for this.
* The most recent and "best" working version of this package corresponds to the most recent commit of this branch.

# Setup

## General Setup:
* Make sure to have steps (1) till (7) of the simulation setup guide finished. The aforementioned setup guide can be found inside the README of 'simulation_code_jan' branch of this repository.
* For step (5), copy the contents the folder in this branch inside your catkin workspace (in the src folder)
* Make sure that at the end of all these step your ROS_MASTER_URI variable inside .bashrc is set to locobot(i.e. the file should contain the following line: export ROS_MASTER_URI=ht<span>tps://<span>locobot.local:1131 or export ROS_MASTER_URI=ht<span>tp://<span>134.91.77.188:11311 if you are using a personal PC/laptop).

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
