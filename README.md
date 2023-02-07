# Masterprojekt-Intelligente-Systeme

This is the final version of the code that was used for the simulation


Steps for setup:

Operating system: Ubuntu 20.04

For prediction model: compatible GPU with CUDA drivers

1. Setup ROS noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
2. Setup catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
3. Make sure Gazebo is installed. If not: https://gazebosim.org/docs/garden/install_ubuntu
4. Install the interbotix locobot package: Run the commands under "Control Software Installation" but replace "create3" with "kobuki"
  https://www.trossenrobotics.com/docs/interbotix_xslocobots/getting_started/user_guide.html
5. Copy the pedsim_dwa and pedsim_ros folders to your catkin workspace (in the src folder)
6. Run catkin_make in your catkin workspace. If you have problems sourcing the two workspaces, a solution can be found here: https://answers.ros.org/question/205976/sourcing-from-multiple-workspaces/
7. pip install -r requirements.txt

(depending on the cofiguration, the package python-is-python3 could be neccessary) (run: sudo apt-get install python-is-python3)

Setup finished!

To run the simulation:
e.g.: "roslaunch pedsim_gazebo_plugin proj_main.launch"

To start the navigation:
"roslaunch pedsim_dwa dwa.launch"

More information in the READMEs in the Folders
  
