#!/usr/bin/env python3
import sys

import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from pedsim_msgs.msg import AgentStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from config import ConfigRobot, ConfigSim
from DWA import DWA
from datetime import datetime, timedelta
import time

import numpy as np


class MainNode:
    def __init__(self, hist, num_agent, r_goal, fov):
        # current state and goal of robot
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        self.r_goal = np.array(list(map(float, r_goal.split())))  # goal_x, goal_y
    
    
    
    	#enable/disable emergency_stop at certain lidar range
        self.enable_emergency_stop = False
        self.emergency_stop = False
        self.stop_range = 0.5
        
        
        #parameters for collecting simulation data:
        self.enable_data_collection = False
        self.min_dist = 1000
        self.time_to_goal = 0
        self.start_pos = np.array([0,0])
        self.dist_to_goal = np.linalg.norm(self.r_goal - self.start_pos)
        self.start_time = datetime.now()
        self.robot_pos_tick_time = datetime.now()
        self.robot_pos = list()      
        self.robot_pos.append(self.start_pos)
        

    
        self.num_agent = num_agent
        self.cmd_vel = Twist()
        self.hist = hist
        self.fov = math.radians(fov)
        
        
        self.min_measurements = 25
        
        self.scan_steps = 10
        
        
        # calculate neighboring index(diff. representation of an adjacency matrix
        self.neigh_idx = np.ones((self.num_agent * (self.num_agent - 1), 3), dtype=np.int)
        k = 0
        for i in range(self.num_agent):
            for j in range(self.num_agent):
                if i != j:
                    self.neigh_idx[k, :] = np.array([i, j, j])
                    k += 1

        # current position of all pedestrians
        self.ped_pos_xy = np.ones((hist+1, self.num_agent, 2)) * (-1000)
        
  

        # current and last 2 positions of robot
        self.r_pos_xy = np.array([0,0])
        self.r_pos_xy_last = np.array([0,0])
        self.r_pos_xy_last_last = np.array([0,0])
        
        

        # configurations for the Robot and Simulation
        self.robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
        self.sim_config = ConfigSim(robot_config=self.robot_config)

        # initialize DWA, once we know the number of agents
        self.dwa = DWA(sim_config=self.sim_config, hist=self.hist, num_agent=self.num_agent, neigh_index=self.neigh_idx)
        print("DWA initialized with  %d number of agents" % ( self.num_agent))

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=10)



        # Subscribers
        ped_sub = message_filters.Subscriber("pedsim_simulator/simulated_agents", AgentStates)
        robot_sub = message_filters.Subscriber("pedsim_simulator/robot_position", Odometry)
        lidar_sub = message_filters.Subscriber("/locobot/scan", LaserScan)
        ts = message_filters.ApproximateTimeSynchronizer([ped_sub, robot_sub, lidar_sub],queue_size=1, slop = 0.1)
        
        ts.registerCallback(self.call_robot)

        rospy.spin()

    
    def laserToAbs(self, scan, x_base, y_base, angle_base):
    
        """
        Converts the lidar measurements (distances and angles) to absolute coordinate in the world coordinate system

        """
        self.emergency_stop = False
        pos_abs = np.ones((int(len(scan.ranges)/ self.scan_steps) , 2)) * np.inf * (-1)
        for i in range(0, len(scan.ranges), self.scan_steps):
            if(scan.ranges[i] < self.stop_range and self.enable_emergency_stop): #for emergency stop
                self.emergency_stop = True
            if(scan.ranges[i] < self.min_dist) and self.enable_data_collection: #for collecting simulation data
                self.min_dist = scan.ranges[i]
            if(scan.ranges[i] > self.sim_config.inner_proximity_radius): #only use measurements in the lidar range
                x = float('inf') * (-1)#-1000
                y = float('inf') * (-1)#-1000
            else: #conversion from from ranges/angles to coordinates
                angle = angle_base - scan.angle_min + (i * scan.angle_increment)
                y = y_base - (math.sin(angle) * scan.ranges[i])
                x = x_base -  (math.cos(angle) * scan.ranges[i])

            pos_abs[int(i/self.scan_steps), 0] = x
            pos_abs[int(i/self.scan_steps), 1] = y

        return pos_abs


    def inFOV(self, ob: np.ndarray, r_pos: np.ndarray, r_norm: np.ndarray, fov_angle: float) -> bool:
        """
        Decides whether an obstacle is in FOV of robot or not. \n
        :param ob: position of the obstacle
        :param r_pos: 2D array, showing the position of the robot
        :param r_norm: unit directional vector of robot
        :param fov_angle: field of view angle in radians given as floating point number
        :return: true if obstacle is in FOV of the robot, and false otherwise
        """
        # vector from robot to obstacle
        r_to_ped = np.array([ob[0] - r_pos[0], ob[1] - r_pos[1]], dtype=np.float)

        # scalar product of r_to_ped and r_norm
        dot_prod = np.dot(r_norm, r_to_ped)

        # calc. angle between two vectors
        phi = math.acos(dot_prod / (1.0 * np.linalg.norm(r_to_ped)))

        # FOV check
        if abs(phi) <= fov_angle / 2:
            return True

        return False

    def save_and_new_goal(self):
        """
        Selects a new goal for the robot, and adjusts all neccesary parameter for data collection

        For purpose of data collection 
        
        !!only made for the proj_main scenario!!
        """

        
        data = "Start_pos: " + str(self.start_pos) + "   Goal_pos: " + str(self.r_goal) + "   Dist_to_goal: " + str(self.dist_to_goal) + "   Min_dist: " + str(self.min_dist) + "   Time: " + str(datetime.now() - self.start_time) + "   Robot_positions: " + str(self.robot_pos)
        
        with open('/home/jan/catkin_ws/src/pedsim_dwa/data.txt', 'a') as f:
            f.write('\n' + data)
            f.close
            
        if self.r_goal[1] == 0: #manually select new goal --> only for proj_main.launch
            self.r_goal = np.array([27,27])
        else:
            self.r_goal = np.array([0,0])

        self.start_time = datetime.now() 
        self.start_pos = self.r_pos_xy    
        self.robot_pos = list()
        self.robot_pos.append(self.r_pos_xy)        
        self.dist_to_goal = np.linalg.norm(self.r_goal - self.r_pos_xy)            
        self.min_dist = 1000         
           



        



    def call_robot(self,ped_msg, robot_msg, lidar_sub):
    
	#update the current and the last 2 robot positions
        self.r_pos_xy_last_last = self.r_pos_xy_last
        self.r_pos_xy_last = [robot_msg.pose.pose.position.x - self.r_pos_xy[0], robot_msg.pose.pose.position.y - self.r_pos_xy[1]]
        self.r_pos_xy = [robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y]

	#calculate the robot angle foor the lidar function
        t1 = 2 * (robot_msg.pose.pose.orientation.w * robot_msg.pose.pose.orientation.z + robot_msg.pose.pose.orientation.x * robot_msg.pose.pose.orientation.y)
        t2 = 1 - 2 * (robot_msg.pose.pose.orientation.y * robot_msg.pose.pose.orientation.y + robot_msg.pose.pose.orientation.z * robot_msg.pose.pose.orientation.z)
        yaw = math.atan2(t1, t2)
        lidar_abs= self.laserToAbs(lidar_sub, robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y, yaw)
        
        lidar_abs = np.expand_dims(lidar_abs, 0)


        #once per second save the position of the robot for data collection
        if datetime.now() - self.robot_pos_tick_time  > timedelta(seconds=1) and self.enable_data_collection: 
            self.robot_pos_tick_time = datetime.now()
            self.robot_pos.append(self.r_pos_xy)

        # unit vector at origin, with the same direction as the robot
        r_norm = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float)



        for agent_idx, agent in enumerate(ped_msg.agent_states):
            if self.inFOV([agent.pose.position.x, agent.pose.position.y], self.r_pos_xy, r_norm, self.fov):
                for hist_idx, h in enumerate(agent.past_pos):
                    self.ped_pos_xy[hist_idx, agent_idx, :] = h.x, h.y
            else:
               self.ped_pos_xy[:, agent_idx, :] = -1000, -1000#float('inf') * (-1), float('inf') * (-1)




        #if robot is close enough to goal --> Done
        if (np.linalg.norm(self.r_goal - self.r_pos_xy) <= self.sim_config.robot_radius):
             print("Done")
             if self.enable_data_collection:
                 self.save_and_new_goal()
                 print(self.r_goal)
             else:
                 self.pub.publish(Twist()) #empty command to stop
                 

        else:
            _, last_yaw = self.dwa.cart2pol(self.r_pos_xy_last_last[0], self.r_pos_xy_last_last[1])
            v, yaw = self.dwa.cart2pol(self.r_pos_xy_last[0], self.r_pos_xy_last[1])
		    # TODO: make cycle rate a param(and put instead of 0.1)
            v_t = v / 0.04
            omega_t = (yaw - last_yaw) / 0.04
            v_t = np.clip(v_t, a_min=self.robot_config.robot_params['min_linear_vel'],
		                  a_max=self.robot_config.robot_params['max_linear_vel'])
            omega_t = np.clip(omega_t, a_min=-self.robot_config.robot_params['max_angular_vel'],
		                      a_max=self.robot_config.robot_params['max_angular_vel'])
            v_x, v_y = self.dwa.pol2cart(v_t, omega_t)

		    # set robot state
            self.r_state[0] = v_x
            self.r_state[1] = v_y
            self.r_state[2] = yaw
            self.r_state[3] = v_t
            self.r_state[4] = omega_t




		    
            u = self.dwa.predict(obs_traj_pos=self.ped_pos_xy, obs_lidar = lidar_abs,  r_pos_xy=self.r_pos_xy,
		                         r_goal_xy=self.r_goal, r_vel_state=self.r_state)

            # Create a Twist message and add linear x and angular z values
            self.cmd_vel.linear.x = u[0, 0]
            self.cmd_vel.angular.z = u[0, 1]

            if self.enable_emergency_stop:
                if(self.emergency_stop == False):
                    self.pub.publish(self.cmd_vel)
                else:
                    self.pub.publish(Twist())
            else:
                self.pub.publish(self.cmd_vel)





if __name__ == '__main__':
    # get command-line arguments
    hist = 7
    num_agent = rospy.get_param('/pedsim_dwa/num_agents')
    r_goal = rospy.get_param('/pedsim_dwa/robot_goal')
    fov = rospy.get_param('/pedsim_dwa/fov')

    # initialize ros node
    rospy.init_node("pedsim_dwa", anonymous=True)

    try:
        main_node = MainNode(hist, num_agent, r_goal, fov)
    except rospy.ROSInternalException:
        rospy.loginfo("Exit due to exception")
        pass
