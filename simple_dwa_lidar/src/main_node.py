#!/usr/bin/env python3
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
    Main Node class to manage all information
    """
    def __init__(self, num_agent, r_goal):
        """
        Constructor to initialize all parameters.\n
        :param num_agent: max. number of agents to keep track of
        :param r_goal: goal of the robot given as
        """
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        self.r_goal = np.array(list(map(float, r_goal.split())))  # goal_x, goal_y
        self.num_agent = num_agent
        self.cmd_vel = Twist()

        # LiDAR related params
        self.min_measurements = 25
        self.scan_steps = 10
        self.min_dist = 1000

        # current position, current and last displacement of robot
        self.r_pos_xy = np.array([0, 0])
        self.r_pos_xy_rel = np.array([0, 0])
        self.r_pos_xy_rell = np.array([0, 0])

        # configurations for the Robot and Simulation
        self.robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
        self.sim_config = ConfigNav(robot_config=self.robot_config)

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
        """
        Function to iterate over and convert LiDAR data into world coordinates
        :param scan: LaserScan measurement from LiDAR
        :param x_base: position of the robot in x-axis
        :param y_base: position of the robot in y-axis
        :param angle_base: rotation of the robot around z-axis
        :return: numpy array containing world coordinates of obstacles
        """
        pos_abs = np.ones((int(len(scan.ranges) / self.scan_steps), 2)) * np.inf * (-1)
        for i in range(0, len(scan.ranges), self.scan_steps):
            if (scan.ranges[i] < self.min_dist):
                self.min_dist = scan.ranges[i]

            if (scan.ranges[i] > self.sim_config.inner_proximity_radius or scan.ranges[i] == 0 or scan.ranges[
                i] == inf):
                x = float('inf') * (-1)
                y = float('inf') * (-1)
            else:
                angle = angle_base - scan.angle_min + (i * scan.angle_increment)  # teset
                y = y_base - (math.sin(angle) * scan.ranges[i])
                x = x_base - (math.cos(angle) * scan.ranges[i])

            pos_abs[int(i / self.scan_steps), 0] = x
            pos_abs[int(i / self.scan_steps), 1] = y

        return pos_abs

    def call_robot(self, robot_msg, lidar_sub):
        """
        Callback function to work on the LiDAR data as well as the current position of the robot\n
        :param robot_msg: Odometry information of the robot
        :param lidar_sub: LaserScan measurements from the LiDAR
        """
        # save last displacement
        self.r_pos_xy_rell = self.r_pos_xy_rel

        # save current displacement
        self.r_pos_xy_rel = [robot_msg.pose.pose.position.x - self.r_pos_xy[0],
                             robot_msg.pose.pose.position.y - self.r_pos_xy[1]]

        # update current position
        self.r_pos_xy = [robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y]

        # convert quaternion to rotation in z-axis
        t1 = 2 * (
                    robot_msg.pose.pose.orientation.w * robot_msg.pose.pose.orientation.z + robot_msg.pose.pose.orientation.x * robot_msg.pose.pose.orientation.y)
        t2 = 1 - 2 * (
                    robot_msg.pose.pose.orientation.y * robot_msg.pose.pose.orientation.y + robot_msg.pose.pose.orientation.z * robot_msg.pose.pose.orientation.z)
        yaw = math.atan2(t1, t2)

        # convert lidar values to absolute values
        lidar_abs = self.laserToAbs(lidar_sub, robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y, yaw)
        lidar_abs = np.expand_dims(lidar_abs, 0)

        if np.linalg.norm(self.r_goal - self.r_pos_xy) <= self.sim_config.robot_radius:
            # if we have reached the goal, stop
            self.pub.publish(Twist())
        else:
            # rotation in z-axis in last tick
            _, last_yaw = self.dwa.cart2pol(self.r_pos_xy_rell[0], self.r_pos_xy_rell[1])

            # current rotation in z-axis in last tick and linear velocity
            v, yaw = self.dwa.cart2pol(self.r_pos_xy_rel[0], self.r_pos_xy_rel[1])

            # calculate velocities from displacements
            v_t = v / 0.04
            omega_t = (yaw - last_yaw) / 0.04

            # make sure current velocities are within correct range
            v_t = np.clip(v_t, a_min=self.robot_config.robot_params['min_linear_vel'],
                          a_max=self.robot_config.robot_params['max_linear_vel'])
            omega_t = np.clip(omega_t, a_min=-self.robot_config.robot_params['max_angular_vel'],
                              a_max=self.robot_config.robot_params['max_angular_vel'])

            # convert velocities to cartesian equivalents
            v_x, v_y = self.dwa.pol2cart(v_t, omega_t)

            # set robot state
            self.r_state[0] = v_x
            self.r_state[1] = v_y
            self.r_state[2] = yaw
            self.r_state[3] = v_t
            self.r_state[4] = omega_t

            # calculate best proposed action using DWA
            u = self.dwa.predict(obs_traj_pos=-1, obs_lidar=lidar_abs, r_pos_xy=self.r_pos_xy,
                                 r_goal_xy=self.r_goal, r_vel_state=self.r_state)

            # Create a Twist message and add linear x and angular z values
            self.cmd_vel.linear.x = u[0, 0]
            self.cmd_vel.angular.z = u[0, 1]

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
