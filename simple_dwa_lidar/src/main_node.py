#!/usr/bin/env python3
"""
    Main Script of the Package responsible for starting and managing the ROS Node.
"""
import sys

import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from config import ConfigRobot, ConfigNav
from DWA import DWA
from datetime import datetime, timedelta

import numpy as np
from numpy import inf


class MainNode:
    """
        Class that manages the Ros Node(Subscribers, Publishers etc.) as well as call the necessary auxiliary functions
        such as the DWA
    """
    def __init__(self, num_agent: str, r_goal: str):
        """
        Simple Constructor to initialize relevant parameters.\n
        :param num_agent: max. number of agents to keep track of
        :param r_goal: goal of the robot given as x, y position in world coordinates
        """
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        self.r_goal = np.array(list(map(float, r_goal.split())))  # goal_x, goal_y
        self.num_agent = int(num_agent)
        self.cmd_vel = Twist()

        # LiDAR parameters
        self.scan_steps = 10  # take every 10 measurement of the LiDAR into account

        # current position of the robot
        self.r_pos_xy = np.array([0, 0])  # [m]

        # displacement of the robot since last tick
        self.r_pos_xy_rel = np.array([0, 0])  # [m]

        # displacement of the robot in the previous tick
        self.r_pos_xy_rel_prev = np.array([0, 0])  # [m]

        # configurations for the Robot and Simulation
        self.robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.35)
        self.nav_config = ConfigNav(robot_config=self.robot_config)

        # initialize DWA, once we know the number of agents
        self.dwa = DWA(nav_config=self.nav_config, num_agent=self.num_agent)
        print("DWA initialized with  %d number of agents" % self.num_agent)

        # Publishers
        self.pub = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=10)

        # Subscribers
        robot_sub = message_filters.Subscriber("/locobot/mobile_base/odom", Odometry)
        lidar_sub = message_filters.Subscriber("/locobot/scan", LaserScan)

        # Synchronize the subscribers
        ts = message_filters.ApproximateTimeSynchronizer([robot_sub, lidar_sub], queue_size=1, slop=0.1)
        ts.registerCallback(self.call_robot)

        rospy.spin()

    def laserToAbs(self, scan: LaserScan, x_base: float, y_base: float, angle_base: float) -> np.ndarray:
        """
        Function to convert LiDAR data to world coordinates.
        :param scan: LaserScan data from LiDAR
        :param x_base: current x-coordinate of the robot as floating point number
        :param y_base: current y-coordinate of the robot as floating point number
        :param angle_base: current orientation in z-axis of the robot as floating point number
        :return: detected obstacles from LiDAR as Numpy Array of size: (no. scans / scan_steps, 2)
        """
        pos_abs = np.ones((int(len(scan.ranges) / self.scan_steps), 2)) * np.inf * (-1)
        for i in range(0, len(scan.ranges), self.scan_steps):
            # if the object is too far away(or the value is a default one like 0 or inf), set its value to -inf
            if scan.ranges[i] > self.nav_config.inner_proximity_radius or scan.ranges[i] == 0 or scan.ranges[i] == inf:
                x = float('inf') * (-1)
                y = float('inf') * (-1)
            else:
                # convert angle to world coordinates
                angle = angle_base - scan.angle_min + (i * scan.angle_increment)
                y = y_base - (math.sin(angle) * scan.ranges[i])
                x = x_base - (math.cos(angle) * scan.ranges[i])

            # set the values inside the array
            pos_abs[int(i / self.scan_steps), 0] = x
            pos_abs[int(i / self.scan_steps), 1] = y

        return pos_abs

    def call_robot(self, robot_msg: Odometry, lidar_msg: LaserScan):
        """
        Function to work on LiDAR and robot Odometry information, call the DWA as well as publish the best proposed
        action to the robot\n
        :param robot_msg: position and orientation of the robot as Odometry information
        :param lidar_msg: LiDAR scan as LaserScan information
        :return: -
        """
        # set the last displacement
        self.r_pos_xy_rel_prev = self.r_pos_xy_rel

        # calculate the displacement of the robot since the last tick
        self.r_pos_xy_rel = [robot_msg.pose.pose.position.x - self.r_pos_xy[0],
                             robot_msg.pose.pose.position.y - self.r_pos_xy[1]]

        # set the new current position of the robot
        self.r_pos_xy = [robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y]

        # calculate the orientation in z-axis from quaternion information
        t1 = 2 * (robot_msg.pose.pose.orientation.w * robot_msg.pose.pose.orientation.z +
                  robot_msg.pose.pose.orientation.x * robot_msg.pose.pose.orientation.y)
        t2 = 1 - 2 * (robot_msg.pose.pose.orientation.y * robot_msg.pose.pose.orientation.y +
                      robot_msg.pose.pose.orientation.z * robot_msg.pose.pose.orientation.z)
        yaw = math.atan2(t1, t2)

        # convert LiDAR data into world coordinates
        lidar_abs = self.laserToAbs(lidar_msg, robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y, yaw)

        # set the shape for correct cost calculation
        lidar_abs = np.expand_dims(lidar_abs, 0)

        # if we are close enough to the goal stop (+0.2 added after testing the robot in real world)
        if np.linalg.norm(self.r_goal - self.r_pos_xy) <= (self.nav_config.robot_radius + 0.2):
            self.pub.publish(Twist())
        else:
            # calculate change in angle in last tick
            _, last_yaw = self.dwa.cart2pol(self.r_pos_xy_rel_prev[0], self.r_pos_xy_rel_prev[1])

            # calculate current change in angle and linear velocity since last tick
            v, yaw = self.dwa.cart2pol(self.r_pos_xy_rel[0], self.r_pos_xy_rel[1])

            # calculate current linear and angular velocity
            v_t = v / 0.04
            omega_t = (yaw - last_yaw) / 0.04

            # make sure that the values are within the allowed values
            v_t = np.clip(v_t, a_min=self.robot_config.robot_params['min_linear_vel'],
                          a_max=self.robot_config.robot_params['max_linear_vel'])
            omega_t = np.clip(omega_t, a_min=-self.robot_config.robot_params['max_angular_vel'],
                              a_max=self.robot_config.robot_params['max_angular_vel'])

            # convert the velocity information (from polar) into cartesian
            v_x, v_y = self.dwa.pol2cart(v_t, omega_t)

            # set robot state
            self.r_state[0] = v_x
            self.r_state[1] = v_y
            self.r_state[2] = yaw
            self.r_state[3] = v_t
            self.r_state[4] = omega_t

            # DWA calculates the best proposed action
            best_action = self.dwa.predict(obs_lidar=lidar_abs, r_pos_xy=np.array(self.r_pos_xy), r_goal_xy=self.r_goal,
                                           r_vel_state=self.r_state)

            # create a Twist message and add linear x and angular z values
            self.cmd_vel.linear.x = best_action[0, 0]
            self.cmd_vel.angular.z = best_action[0, 1]

            # publish the message to the robot
            self.pub.publish(self.cmd_vel)


if __name__ == '__main__':
    # get command-line arguments
    num_agent = rospy.get_param('/simple_dwa_lidar/num_agents')
    r_goal = rospy.get_param('/simple_dwa_lidar/robot_goal')

    # initialize ros node
    rospy.init_node("simple_dwa_lidar", anonymous=True)

    try:
        main_node = MainNode(num_agent, r_goal)
    except rospy.ROSInternalException:
        rospy.loginfo("Exit due to exception")
        pass
