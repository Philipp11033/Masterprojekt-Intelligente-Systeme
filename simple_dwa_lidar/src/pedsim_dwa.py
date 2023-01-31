#!/usr/bin/env python3
import sys

import rospy
import message_filters
import math
import numpy as np
from numpy import inf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from config import ConfigRobot, ConfigNav
from DWA import DWA


class MainNode:
    """
    Main Node class to manage all information
    """
    def __init__(self, num_obs, r_goal_xy):
        """
        Constructor to initialize all parameters.\n
        :param num_obs: max. number of agents to keep track of
        :param r_goal_xy: goal of the robot given as
        """
        # goal_x, goal_y
        self.r_goal = np.array(r_goal_xy)

        # max. number of obstacles to keep track of at any given time
        self.num_agent = num_obs

        # twist message to send to the robot
        self.cmd_vel = Twist()

        # current state of robot to be initialized later : [v_x(m/s), v_y(m/s), theta(rad), v_lin(m/s), v_ang(rad/s)]
        self.r_state = np.zeros(5)

        ### Parameters for LiDAR
        # no. measurements to skip when considering all the measurements
        self.scan_steps = 10

        ### Parameters for tracking position of the robot
        # current position of robot
        self.r_pos_xy = np.array([0, 0])

        # displacement of the robot
        self.r_pos_xy_rel = np.array([0, 0])

        # additional array to keep track of the previous displacement of the robot
        self.r_pos_xy_rel_prev = np.array([0, 0])


        ### Init objects, publishers and subscribers
        # configurations for the Robot and Navigation
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

        # synchronization of the subscribers
        ts = message_filters.ApproximateTimeSynchronizer([robot_sub, lidar_sub], queue_size=1, slop=0.1)
        ts.registerCallback(self.call_robot)

        rospy.spin()

    def laser_to_abs(self, scan, x_base, y_base, angle_base):
        """
        Function to iterate over and convert LiDAR data into world coordinates
        :param scan: LaserScan measurement from LiDAR
        :param x_base: position of the robot in x-axis
        :param y_base: position of the robot in y-axis
        :param angle_base: rotation of the robot around z-axis
        :return: numpy array containing world coordinates of obstacles
        """
        # placeholder array for data
        pos_abs = np.ones((int(len(scan.ranges)/self.scan_steps), 2)) * np.inf * (-1)

        for i in range(0, len(scan.ranges), self.scan_steps):
            # skip any possible default values
            if scan.ranges[i] > self.sim_config.inner_proximity_radius or scan.ranges[i] == 0 or scan.ranges[i] == inf:
                x = float('inf') * (-1)
                y = float('inf') * (-1)
            else:
                # convert scans to cartesian coord. values
                angle = angle_base - scan.angle_min + (i * scan.angle_increment)
                y = y_base - (math.sin(angle) * scan.ranges[i])
                x = x_base - (math.cos(angle) * scan.ranges[i])

            pos_abs[int(i/self.scan_steps), 0] = x
            pos_abs[int(i/self.scan_steps), 1] = y

        return pos_abs

    def call_robot(self, robot_msg, lidar_sub):
        """
        Callback function to work on the LiDAR data as well as the current position of the robot\n
        :param robot_msg: Odometry information of the robot
        :param lidar_sub: LaserScan measurements from the LiDAR
        """
        # save previous displacement
        self.r_pos_xy_rel_prev = self.r_pos_xy_rel

        # calc. curr displacement
        self.r_pos_xy_rel = [robot_msg.pose.pose.position.x - self.r_pos_xy[0], robot_msg.pose.pose.position.y - self.r_pos_xy[1]]

        # save curr position
        self.r_pos_xy = [robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y]

        # conversion of quaternion to rotation in z-axis
        t1 = 2 * (robot_msg.pose.pose.orientation.w * robot_msg.pose.pose.orientation.z + robot_msg.pose.pose.orientation.x * robot_msg.pose.pose.orientation.y)
        t2 = 1 - 2 * (robot_msg.pose.pose.orientation.y * robot_msg.pose.pose.orientation.y + robot_msg.pose.pose.orientation.z * robot_msg.pose.pose.orientation.z)
        yaw = math.atan2(t1, t2)

        # convert LiDAR data to absolute coordinates
        lidar_abs = self.laser_to_abs(lidar_sub, robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y, yaw)
        lidar_abs = np.expand_dims(lidar_abs, 0)

        # check to make sure goal is reached
        if np.linalg.norm(self.r_goal - self.r_pos_xy) <= self.sim_config.robot_radius:
            self.pub.publish(Twist())
        else:
            # find the direction robot was facing at the last tick
            _, last_yaw = self.dwa.cart2pol(self.r_pos_xy_rel_prev[0], self.r_pos_xy_rel_prev[1])

            # current linear displacement of the robot as well as the current rotation
            v, yaw = self.dwa.cart2pol(self.r_pos_xy_rel[0], self.r_pos_xy_rel[1])

            # convert displacements to velocities
            v_t = v / 0.04
            omega_t = (yaw - last_yaw) / 0.04

            # make sure that the velocities can not go out of possible bounds
            v_t = np.clip(v_t, a_min=self.robot_config.robot_params['min_linear_vel'],
                          a_max=self.robot_config.robot_params['max_linear_vel'])
            omega_t = np.clip(omega_t, a_min=-self.robot_config.robot_params['max_angular_vel'],
                              a_max=self.robot_config.robot_params['max_angular_vel'])

            # convert velocities to cartesian coord.
            v_x, v_y = self.dwa.pol2cart(v_t, omega_t)

            # set the current robot state
            self.r_state[0] = v_x
            self.r_state[1] = v_y
            self.r_state[2] = yaw
            self.r_state[3] = v_t
            self.r_state[4] = omega_t

            best_action = self.dwa.predict(obs_traj_pos=-1, obs_lidar=lidar_abs,  r_pos_xy=self.r_pos_xy,
                                           r_goal_xy=self.r_goal, r_vel_state=self.r_state)

            # set linear x and angular z values
            self.cmd_vel.linear.x = best_action[0, 0]
            self.cmd_vel.angular.z = best_action[0, 1]

            self.pub.publish(self.cmd_vel)


if __name__ == '__main__':
    # get command-line arguments
    # num_obs = rospy.get_param('/pedsim_dwa/num_agents')
    # r_goal_xy = rospy.get_param('/pedsim_dwa/robot_goal')
    num_agent = 10
    r_goal = [5.5, 0.0]

    # initialize ros node
    rospy.init_node("simple_dwa_lidar", anonymous=True)

    try:
        main_node = MainNode(num_agent, r_goal)
    except rospy.ROSInternalException:
        rospy.loginfo("Exit due to exception")
        pass
