# DWA+LiDAR
This branch contains the version of the code that uses only DWA and LiDAR for navigation. We refer to this version of the code simply as DWA+LiDAR. Its usage is explained in the README of the 'real_robot_utku' branch of this repository, but for ease of access, it is appended below as well.

### DWA+LiDAR
* Name of the ros package: simple_dwa_lidar
* This version of the code has a straightforward DWA implementation as a local planner and uses LiDAR data for collision avoidance. The best proposed actions from the DWA are sent to the locobot as Twist messages. For localization, locobot's own localization is used.
* This package has been tested and can be used for navigation of the robot (remark: setting the most suitable weighting parameters for cost functions, LiDAR range as well as max. linear velocity is key for optimal behaviour of the algorithm. The current best parameters have already been set in the latest commit of the respective branch).

# Setup

## General Setup:
* Make sure to have steps (1) till (7) of the simulation setup guide finished. The aforementioned setup guide can be found inside the README of 'simulation_code_jan' branch of this repository.
* For step (5), copy the contents the folder in this branch inside your catkin workspace (in the src folder)
* Make sure that at the end of all these step your ROS_MASTER_URI variable inside .bashrc is set to locobot(i.e. the file should contain the following line: export ROS_MASTER_URI=ht<span>tps://<span>locobot.local:1131 or export ROS_MASTER_URI=ht<span>tp://<span>134.91.77.188:11311 if you are using a personal PC/laptop).

## Start Up the Control Package for Locobot:
* Starting the interbotix control package requires an ssh connection to the locobot(using any PC at the LAB BC422 or any laptop with access to one of the follwing WLANs: IS-Stud, LAB_BC_422).
* Once, you are connected to the locobot, run: 
```
roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_base use_base:=true use_lidar:=true use_camera:=false
```

# Running The simple_dwa_lidar Package
* Requires the control package from Locobot to be running.
* We suggest setting the parameters in config file accordingly: to_goal_cost = 0.5, speed_cost_gain = 10, obstacle_cost_gain = 3, pred_time_steps = 4 and max_linear_vel(inside Config Robot Class) to .3 m/s
* From remote PC, run(remark: the robot_goal and num_agent are free parameters that can be initialized as shown below, each indicating the goal of the robot in world coordinates and the max. number of obstacles being tracked respectively)
```
roslaunch simple_dwa_lidar demo.launch robot_goal:="5.5 0.0" num_agent:="30"
```

# Additional Useful Commands
* For controlling the locobot using keyboards, once can run the following from the terminal of the locobot(remark: requires the control package to be running):
```
roslaunch kobuki_keyop keyop.launch __ns:=locobot
```
* For reseting the odometry of the locobot(remark: requires the control package to be running):
```
rostopic pub --once /locobot/mobile_base/commands/reset_odometry std_msgs/Empty "{}"
```
