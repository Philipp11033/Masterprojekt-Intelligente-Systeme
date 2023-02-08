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
* The current most stable version of the code is 61c3e32708431febdffa11dd7104ada7d1655344
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

## Start Up the Control Package for Locobot:
* Starting the interbotix control package requires an ssh connection(using any PC at the LAB BC422 or any laptop with access to one of the follwing WLANs: IS-Stud, LAB_BC_422).
* Once, you are connected to the locobot, run: 
```
roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_base use_base:=true use_lidar:=true use_camera:=true
```

## Start Up the Robot Perception Package:
* Starting the robot perception package requires either an ssh connection(using any PC at the LAB BC422 or any laptop with access to one of the follwing WLANs: IS-Stud, LAB_BC_422) or direct access to the ZedBox through an HDMI cabel and a monitor. 
* Once, inside ZedBox, make sure to set the date and time of the zedBox to exactly same as the locobot(relevant ubuntu commands are: date and timedatectl set-time hh:mm:ss)
* Finally, run: 
```
roslaunch praxis_projekt_ws22 robot_perception.launch
```

# Running The simple_dwa_lidar Package
* Requires the control package from Locobot
* We suggest setting the parameters in config file accordingly: to_goal_cost = 0.5, speed_cost_gain = 10, obstacle_cost_gain = 3, pred_time_steps = 4 and max_linear_vel(inside Config Robot Class) to .3 m/s
* From remote PC, run
```
roslaunch simple_dwa_lidar demo.launch
```

# Running The dwa_w_prediction Package
* Requires the control package from Locobot as well as the robot perception package
* From remote PC, run
```
roslaunch test_robot_perception demo.launch
```
