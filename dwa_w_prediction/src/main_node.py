#!/usr/bin/env python3
"""
    Main Script of the Package responsible for starting and managing the ROS Node.
"""
import datetime
import sys

import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import tf
from tf.transformations import quaternion_matrix
from config import ConfigRobot, ConfigNav
from DWA import DWA

import numpy as np
from numpy import inf


class MainNode:
    """
        Class that manages the Ros Node(Subscribers, Publishers etc.) as well as call the necessary auxiliary functions
        such as the DWA
    """
    def __init__(self, num_agent: str, r_goal: str, hist: int):
        """
        Constructor to initialize all parameters.\n
        :param num_agent: max. number of agents to keep track of
        :param r_goal: goal of the robot given as
        :param hist: number of ticks into the past we keep track of for humans
        """
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        self.r_goal = np.array(list(map(float, r_goal.split())))  # goal_x, goal_y
        self.num_agent = int(num_agent)
        self.hist = hist
        self.cmd_vel = Twist()

        # calculate neighboring index(diff. representation of an adjacency matrix)
        self.neigh_idx = np.ones((self.num_agent * (self.num_agent - 1), 3), dtype=np.int)
        k = 0
        for i in range(self.num_agent):
            for j in range(self.num_agent):
                if i != j:
                    self.neigh_idx[k, :] = np.array([i, j, j])
                    k += 1

        # current position of all pedestrians (default values)
        self.ped_pos_xy_rf_def = np.ones((hist + 1, self.num_agent, 2)) * -1000  # in robot reference frame
        self.ped_pos_xy_wf_def = np.ones((hist + 1, self.num_agent, 2)) * -1000  # in robot world frame

        # LiDAR related params
        self.scan_steps = 10  # take every 10 measurement of the LiDAR into account

        # current position of the robot
        self.r_pos_xy = np.array([0, 0])  # [m]

        # displacement of the robot since last tick
        self.r_pos_xy_rel = np.array([0, 0])  # [m]

        # displacement of the robot in the previous tick
        self.r_pos_xy_rel_prev = np.array([0, 0])  # [m]

        # configurations for the Robot and Simulation
        self.robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
        self.sim_config = ConfigNav(robot_config=self.robot_config)

        # initialize DWA, once we know the number of agents
        self.dwa = DWA(nav_config=self.sim_config, num_agent=self.num_agent, hist=self.hist, neigh_index=self.neigh_idx)
        print("DWA initialized with  %d number of agents" % self.num_agent)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        self.tf_listener = tf.TransformListener()

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
            if scan.ranges[i] > self.sim_config.inner_proximity_radius or scan.ranges[i] == 0 or scan.ranges[i] == inf:
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

    def call_robot(self, robot_msg, lidar_sub):
        """
        Function to work on LiDAR and robot Odometry information, call the DWA as well as publish the best proposed
        action to the robot\n
        :param robot_msg: position and orientation of the robot as Odometry information
        :param lidar_msg: LiDAR scan as LaserScan information
        :return: -
        """
        # receive the interpolated history positions of the people (remark: currently the header part of the Markers
        # inside the MarkerArray are not set, making it not possible to synchronize this topic with the lidar&robot
        # odometry ones. Hence, the time needed for the next message is directly added to the total delay of the code)
        self.data = rospy.wait_for_message("/corrected_interpolated_history_position", MarkerArray, timeout=5)

        # save last displacement
        self.r_pos_xy_rel_prev = self.r_pos_xy_rel

        # save current displacement
        self.r_pos_xy_rel = [robot_msg.pose.pose.position.x - self.r_pos_xy[0],
                             robot_msg.pose.pose.position.y - self.r_pos_xy[1]]

        # update current position
        self.r_pos_xy = [robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y]

        # convert quaternion to rotation in z-axis
        t1 = 2 * (robot_msg.pose.pose.orientation.w * robot_msg.pose.pose.orientation.z +
                  robot_msg.pose.pose.orientation.x * robot_msg.pose.pose.orientation.y)
        t2 = 1 - 2 * (robot_msg.pose.pose.orientation.y * robot_msg.pose.pose.orientation.y +
                      robot_msg.pose.pose.orientation.z * robot_msg.pose.pose.orientation.z)
        yaw = math.atan2(t1, t2)

        # convert lidar values to absolute values and adjust the shape for correct cost calculation
        lidar_abs = self.laserToAbs(lidar_sub, robot_msg.pose.pose.position.x, robot_msg.pose.pose.position.y, yaw)
        lidar_abs = np.expand_dims(lidar_abs, 0)

        # loop is necessary, cause sometimes, the lookupTransform returns a None value. According to our tests, this
        # inner loop introduces a delay of less than 0.001s to the entire code
        while True:
            # extract translation and rotation for robot reference to robot world coordinates
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/locobot/base_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("could not find a look up transform")
                continue


        # convert the rotational vector in quaternion into the rotational matrix in euler angles
        matrix = quaternion_matrix(rot)
        matrix_arr = np.array(matrix)

        # fill the fourth row of the matrix with the trans part of the lookup
        matrix_arr[:, 3] = np.array([trans[0], trans[1], trans[2], 1])

        # current position of all pedestrians (actual values)
        # with np.copy(), they are reset everytime the callback function is called
        self.ped_pos_xy_rf = np.copy(self.ped_pos_xy_rf_def)
        self.ped_pos_xy_wf = np.copy(self.ped_pos_xy_wf_def)

        # extract positions of humans(in robot reference frame)
        for (i, marker) in enumerate(self.data.markers):
            # ignore first entry which belongs to the robot as well as any empty arrays
            if i == 0 or len(marker.points) < 1:
                continue

            for (j, point) in enumerate(marker.points):
                self.ped_pos_xy_rf[j, i - 1, :] = [point.x, point.y]

                # calculate the 4d positional vector(to multiply it with the transformation matrix)
                pos_vec = np.array([point.x, point.y, 0, 1])

                # convert the coordinates to robot world frame
                out = np.matmul(matrix_arr, pos_vec)

                self.ped_pos_xy_wf[j, i - 1, :] = [out[0], out[1]]

            # if array is not full, fill it with the last correct data (often happens when a person is just entering
            # the FOV of the camera)
            if len(marker.points) < 8:
                # find all the valid values
                last_not_default_value = np.where(self.ped_pos_xy_wf[:, i - 1, :] == -1000)

                # extract the latest correct one
                last_corr_xy_value = self.ped_pos_xy_wf[last_not_default_value[0][0]-1, i-1, :]

                # simply fill the rest of the array with it
                self.ped_pos_xy_wf[last_not_default_value[0][0]:, i-1, :] = last_corr_xy_value

        # if we are close enough to the goal stop (+0.2 added after testing the robot in real world)
        if np.linalg.norm(self.r_goal - self.r_pos_xy) <= self.sim_config.robot_radius + 0.2:
            self.pub.publish(Twist())
        else:
            # rotation in z-axis in last tick
            _, last_yaw = self.dwa.cart2pol(self.r_pos_xy_rel_prev[0], self.r_pos_xy_rel_prev[1])

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
            u = self.dwa.predict(obs_traj_pos=self.ped_pos_xy_wf, obs_lidar=lidar_abs, r_pos_xy=np.array(self.r_pos_xy),
                                 r_goal_xy=self.r_goal, r_vel_state=self.r_state)

            # create a Twist message and add linear x and angular z values
            self.cmd_vel.linear.x = u[0, 0]
            self.cmd_vel.angular.z = u[0, 1]

            # publish the message to the robot
            self.pub.publish(self.cmd_vel)


if __name__ == '__main__':
    # get command-line arguments
    num_agent = rospy.get_param('/dwa_w_prediction/num_agents')
    r_goal = rospy.get_param('/dwa_w_prediction/robot_goal')
    hist = rospy.get_param('/dwa_w_prediction/hist')

    # initialize ros node
    rospy.init_node("dwa_w_prediction", anonymous=True)

    try:
        main_node = MainNode(num_agent, r_goal, hist)
    except rospy.ROSInternalException:
        rospy.loginfo("Exit due to exception")
        pass
