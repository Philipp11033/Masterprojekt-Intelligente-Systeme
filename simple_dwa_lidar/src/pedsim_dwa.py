#!/usr/bin/env python3
import sys

import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from config import ConfigRobot, ConfigSim
from DWA import DWA
from datetime import datetime, timedelta


import numpy as np
from numpy import inf

class MainNode:
    def __init__(self, num_agent, r_goal):
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        # self.r_goal = np.array(list(map(float, r_goal.split())))  # goal_x, goal_y

        self.min_dist = 1000
        self.r_goal = np.array(r_goal)

        self.emergency_stop = False
        self.stop_range = 0.5

        self.num_agent = num_agent
        self.cmd_vel = Twist()

        self.min_measurements = 25
        
        self.scan_steps = 10

        # current and last position of robot
        self.r_pos_xy = np.array([0, 0])
        self.r_pos_xy_rel = np.array([0, 0])
        self.r_pos_xy_rell = np.array([0, 0])

        # configurations for the Robot and Simulation
        self.robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
        self.sim_config = ConfigSim(robot_config=self.robot_config)

        # initialize DWA, once we know the number of agents
        self.dwa = DWA(sim_config=self.sim_config, num_agent=self.num_agent)
        print("DWA initialized with  %d number of agents" % (self.num_agent))

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=10)

        # Subscribers
        robot_sub = message_filters.Subscriber("/locobot/mobile_base/odom", Odometry)
        lidar_sub = message_filters.Subscriber("/locobot/scan", LaserScan)
        ts = message_filters.ApproximateTimeSynchronizer([robot_sub, lidar_sub], queue_size=1, slop=0.1)
        
        ts.registerCallback(self.call_robot)

        rospy.spin()

    def laserToAbs(self, scan, x_base, y_base, angle_base):
        #self.emergency_stop = False
        pos_abs = np.ones((int(len(scan.ranges)/self.scan_steps), 2)) * np.inf * (-1)
        for i in range(0, len(scan.ranges), self.scan_steps):
            #if(scan.ranges[i] < self.stop_range):
            #    self.emergency_stop = True
            #    print("stop")
            if(scan.ranges[i] < self.min_dist):
                self.min_dist = scan.ranges[i]
                #print(self.min_dist)
            if(scan.ranges[i] > self.sim_config.inner_proximity_radius or scan.ranges[i] == 0 or scan.ranges[i] == inf):
                x = float('inf') * (-1)#-1000#float('inf')
                y = float('inf') * (-1)#-1000#float('inf')
            else:
                angle = angle_base - scan.angle_min + (i * scan.angle_increment) #teset
                y = y_base - (math.sin(angle) * scan.ranges[i])
                x = x_base - (math.cos(angle) * scan.ranges[i])

            pos_abs[int(i/self.scan_steps), 0] = x
            pos_abs[int(i/self.scan_steps), 1] = y

        return pos_abs

    def call_robot(self, robot_msg, lidar_sub):
        self.r_pos_xy_rell = self.r_pos_xy_rel

        self.r_pos_xy_rel = [robot_msg.pose.pose.position.x - self.r_pos_xy[0], robot_msg.pose.pose.position.y - self.r_pos_xy[1]]
        self.r_pos_xy = [robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y]

        t1 = 2 * (robot_msg.pose.pose.orientation.w * robot_msg.pose.pose.orientation.z + robot_msg.pose.pose.orientation.x * robot_msg.pose.pose.orientation.y)
        t2 = 1 - 2 * (robot_msg.pose.pose.orientation.y * robot_msg.pose.pose.orientation.y + robot_msg.pose.pose.orientation.z * robot_msg.pose.pose.orientation.z)
        yaw = math.atan2(t1, t2)
        lidar_abs= self.laserToAbs(lidar_sub, robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y, yaw)
        
        lidar_abs = np.expand_dims(lidar_abs, 0)

        if (np.linalg.norm(self.r_goal - self.r_pos_xy) <= self.sim_config.robot_radius):
            print("Done")
            print(self.r_goal)
            self.pub.publish(Twist())
        else:
            _, last_yaw = self.dwa.cart2pol(self.r_pos_xy_rell[0], self.r_pos_xy_rell[1])
            v, yaw = self.dwa.cart2pol(self.r_pos_xy_rel[0], self.r_pos_xy_rel[1])
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

            u = self.dwa.predict(obs_traj_pos=-1, obs_lidar = lidar_abs,  r_pos_xy=self.r_pos_xy,
                                 r_goal_xy=self.r_goal, r_vel_state=self.r_state)

            # Create a Twist message and add linear x and angular z values
            self.cmd_vel.linear.x = u[0, 0]
            self.cmd_vel.angular.z = u[0, 1]
            print("\nLiDAR data: ")
            inf_mask = lidar_abs != np.inf * (-1)
            np.set_printoptions(linewidth=35)
            print(lidar_abs[inf_mask])
            # print(lidar_abs)
            # print("Goal: ")
            # print(self.r_goal)
            print("Robot position: ")
            print(self.r_pos_xy)
            # print("Current state of robot: ")
            # print(self.r_state)
            print("Publishing: ")
            print(self.cmd_vel.linear.x, self.cmd_vel.angular.z)
            # self.pub.publish(self.cmd_vel)
            #if(self.emergency_stop == False):
            #    self.pub.publish(self.cmd_vel)
            #else:
            #    self.pub.publish(Twist())


if __name__ == '__main__':
    # get command-line arguments
    # num_agent = rospy.get_param('/pedsim_dwa/num_agents')
    # r_goal = rospy.get_param('/pedsim_dwa/robot_goal')
    num_agent = 10
    r_goal = [5.5, 0.0]

    # initialize ros node
    rospy.init_node("simple_dwa_lidar", anonymous=True)

    try:
        main_node = MainNode(num_agent, r_goal)
    except rospy.ROSInternalException:
        rospy.loginfo("Exit due to exception")
        pass
