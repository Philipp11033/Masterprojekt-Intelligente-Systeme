"""
Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py

Reading the Code and its documentation:
- All coordinates are given/worked upon as world coordinates unless stated otherwise in the documentation/comments.
- Square brackets('[', ']') in comments and documentation refer to a numpy array unless stated otherwise.
"""
import warnings
import math
import numpy as np
from config import ConfigRobot, ConfigNav, RobotType

warnings.filterwarnings("ignore", category=DeprecationWarning)


class DWA:
    """
        Class with a straightforward Dynamic Window Approach implementation.
    """
    def __init__(self, sim_config: ConfigNav, num_agent: int):
        """
        Simple Constructor to initialize relevant parameters.\n
        :param sim_config: configuration object for the chosen robot and navigation params
        :param num_agent: max. number of agents to keep track of
        """
        self.robot_model = sim_config.robot_model
        self.num_agent = num_agent
        self.config = sim_config

    def dwa_control(self, r_curr_state: np.ndarray, r_goal_xy: np.ndarray, obs_xy: np.ndarray):
        """
        Main function for Dynamic Window Approach that calls other necessary functions.\n
        :param r_curr_state: current state of the robot given as: [x(m), y(m), omega(rad), v_r(m/s), v_omega(rad/s)]
        :param r_goal_xy: goal of the robot
        :param obs_xy: current positions of all detected/filtered obstacles
        :return:
        """
        # dynamic window returned here as a list: [v_r_min, v_r_max, v_omega_min, v_omega_max]
        dyn_window = self.calc_dynamic_window(r_curr_state)

        proposed_action, trajectory = self.calc_control_and_trajectory(r_curr_state, dyn_window, r_goal_xy, obs_xy)
        return proposed_action, trajectory

    @staticmethod
    def motion(r_state_in_traj: np.ndarray, proposed_action: list, time_tick: float) -> np.ndarray:
        """
        The motion model for DWA. Calculates the next state of the robot with given parameters.\n
        :param r_state_in_traj: state of the robot for the CURRENT TRAJECTORY UNDER CONSIDERATION given as:
        [x(m), y(m), omega(rad), v_r(m/s), v_omega(rad/s)]
        :param proposed_action: linear(translational) and angular velocity, both in [m/s], given as a list of two floats
        :param time_tick: floating point number to indicate how long into the future we are sampling(see ConfigSim)
        :return: next state of the robot given as: [x(m), y(m), omega(rad), v_r(m/s), v_omega(rad/s)]
        """
        # extract the proposed linear and angular velocities
        prop_lin_vel = proposed_action[0]  # [m/s]
        prop_ang_vel = proposed_action[1]  # [m/s]

        # x_new = v_r * cos(omega) * delta_t, with v_r * cos(omega) = v_x
        r_state_in_traj[0] += prop_lin_vel * math.cos(r_state_in_traj[2]) * time_tick

        # y_new = v_r * sin(omega) * delta_t, with v_r * sin(omega) = v_y
        r_state_in_traj[1] += prop_lin_vel * math.sin(r_state_in_traj[2]) * time_tick

        r_state_in_traj[2] += prop_ang_vel * time_tick  # omega_new = v_omega * delta_t
        r_state_in_traj[3] = prop_lin_vel  # v_r_new = proposed linear velocity
        r_state_in_traj[4] = prop_ang_vel  # v_omega_new = proposed angular velocity

        return r_state_in_traj

    def calc_dynamic_window(self, r_curr_state: np.ndarray) -> list:
        """
        Function to calculate the Dynamic Window based on current state r_curr_state.\n
        :param r_curr_state: current state of the robot given as: [x(m), y(m), omega(rad), v_r(m/s), v_omega(rad/s)]
        :return: Dynamic Window with allowed linear and angular velocity thresholds as a list: [v_r_min, v_r_max, v_omega_min, v_omega_max]
        """

        # Dynamic window from robot specification
        v_allowed = [self.config.min_speed, self.config.max_speed,
                     -self.config.max_yaw_rate, self.config.max_yaw_rate]

        # Dynamic window from motion model
        v_curr_possible = [r_curr_state[3] - self.config.max_accel * self.config.dt,
                           r_curr_state[3] + self.config.max_accel * self.config.dt,
                           r_curr_state[4] - self.config.max_delta_yaw_rate * self.config.dt,
                           r_curr_state[4] + self.config.max_delta_yaw_rate * self.config.dt]

        # Dynamic Window calculated | allowed vel. vs. currently possible vel.
        dyn_window = [max(v_allowed[0], v_curr_possible[0]), min(v_allowed[1], v_curr_possible[1]),
                      max(v_allowed[2], v_curr_possible[2]), min(v_allowed[3], v_curr_possible[3])]

        return dyn_window

    def predict_trajectory(self, r_curr_state: np.ndarray, v_r: float, v_omega: float):
        """
        Predicts the trajectory(array of sequential robot states) within the prediction window.\n
        :param r_curr_state: current state of the robot given as: [x(m), y(m), omega(rad), v_r(m/s), v_omega(rad/s)]
        :param v_r: input linear(translational) velocity
        :param v_omega: input angular velocity
        :return: predicted trajectory(array of all robot states in sequential order)
        """
        # copy current robot state. r_state_in_traj is to be used for iteratively calculating the next state of the
        # robot within the prediction
        r_state_in_traj = np.array(r_curr_state)

        # array to store all states of robot for the current trajectory under consideration
        trajectory = np.array(r_curr_state)

        time = 0
        while time <= self.config.predict_time:
            # calculate next state of the robot inside the prediction
            x = self.motion(r_state_in_traj, [v_r, v_omega], self.config.dt)

            # append to the array of all states
            trajectory = np.vstack((trajectory, x))

            time += self.config.dt

        return trajectory

    @staticmethod
    def calc_to_goal_cost(trajectory: np.ndarray, r_goal_xy: np.ndarray) -> float:
        """
        Calculates the cost to the goal with respect to the angle difference.\n
        :param trajectory: array of all robot states for current trajectory under consideration
        :param r_goal_xy: goal of the robot
        :return:
        """
        # calc distance to goal using last state of robot in pred. frame
        dx = r_goal_xy[0] - trajectory[-1, 0]
        dy = r_goal_xy[1] - trajectory[-1, 1]

        # using arctan, calculate the angle between the current position of the robot and the goal(aka. the optimal
        # angle, the trajectory should have
        optimal_angle = math.atan2(dy, dx)  # [rad]

        # calc the difference between the optimal and current angle
        cost_angle = optimal_angle - trajectory[-1, 2]  # [rad]

        # original implementation's line below, which is equivalent to just taking the absolute value of the cost_angle
        # cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))  # [rad]
        # we are interested in how much the diff. is, not in which direction the difference is -> so, take absolute val.
        cost = abs(cost_angle)

        return cost

    def calc_obstacle_cost(self, trajectory: np.ndarray, obs_xy: np.ndarray):
        """
        Calculates the cost to the goal with respect to the angle difference.\n
        :param trajectory: numpy array of all robot states for "current" prediction(aka. trajectory of robot in prediction)
        :param obs_xy: numpy array of all positions of every obstacle in world coordinate system
        :return: dist of the closest obstacle to the robot (specifically: 1 / min_r)
        """
        # extract x and y positions of all obstacles
        ox = obs_xy[:, :, 0]
        oy = obs_xy[:, :, 1]

        # calculate dist to the robot(in every robot state for the current trajectory) in x and y axis
        dx = trajectory[:, None, 0] - ox
        dy = trajectory[:, None, 1] - oy

        dist_to_obs = np.hypot(dx, dy)

        if self.config.robot_type == RobotType.circle:
            # collision check
            if np.array(dist_to_obs <= self.config.robot_radius).any():
                return float("Inf")
        else:
            raise ValueError("Current implementation of the code only works with circular robot type!")

        # if no collision, choose the obstacle closest to the robot
        min_r = np.nanmin(dist_to_obs)

        return 1.0 / min_r  # OK

    def calc_control_and_trajectory(self, r_curr_state: np.ndarray, dyn_window: list, r_goal_xy: np.ndarray,
                                    obs_xy: np.ndarray):
        """
        Calculates the best proposed action and the trajectory related to that action.\n
        :param r_curr_state: current state of the robot given as: [x(m), y(m), omega(rad), v_r(m/s), v_omega(rad/s)]
        :param dyn_window: Dynamic Window with allowed linear and angular velocity thresholds as a list:
        [v_r_min, v_r_max, v_omega_min, v_omega_max]
        :param r_goal_xy: goal of the robot given as: [x(m), y(m)]
        :param obs_xy: current positions of all detected/filtered obstacles
        :return:
        """
        # init variables
        r_curr_state_init = np.copy(r_curr_state)
        min_cost = float("inf")
        best_proposed_action = [0.0, 0.0]
        best_trajectory = np.array([r_curr_state])

        # evaluate all trajectories with sampled input(acc. resolutions from config.) in dynamic window
        for v_r in np.arange(dyn_window[0], dyn_window[1], self.config.v_resolution):
            for v_omega in np.arange(dyn_window[2], dyn_window[3], self.config.yaw_rate_resolution):

                # calculate predicted trajectory of the robot
                trajectory = self.predict_trajectory(r_curr_state_init, v_r, v_omega)

                ### calc cost of the current "predicted" trajectory

                # calc. the angle difference between the current "predicted" trajectory's angle and the optimal angle
                # to reach the goal (optimal case: robot is looking directly at the goal)
                to_goal_cost = self.config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, r_goal_xy)

                # calc. the difference between the linear velocity of the current "predicted" trajectory and the max
                # allowed linear velocity (optimal case: robot is moving at full speed to the goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])

                # calc. 1 / dist. to closest obstacle
                ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, )

                # total cost of the trajectory
                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_proposed_action = [v_r, v_omega]
                    best_trajectory = trajectory
                    # if current v_r ~ 0 and best v_r ~ 0, robot is stuck (e.g. in front of an obstacle) => make him
                    # turn around, so he can calculate other trajectories
                    if abs(best_proposed_action[0]) < self.config.robot_stuck_flag_cons \
                            and abs(r_curr_state[3]) < self.config.robot_stuck_flag_cons:
                        # to ensure the robot does not get stuck in best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with angle difference of 0)
                        best_proposed_action[1] = -self.config.max_yaw_rate  # Bug in original code? Corrected to max_yaw_rate instead

        return best_proposed_action, best_trajectory

    @staticmethod
    def cart2pol(x: float, y: float) -> tuple:
        """
        Function to convert cartesian coordinates into polar coordinates.\n
        :param x: position in x-axis as floating point number
        :param y: position in y-axis as floating point number
        :return: distance to origin and angle returned as tuple of two floats respectively
        """
        r = np.sqrt(x ** 2 + y ** 2)
        theta = np.arctan2(y, x)
        return r, theta

    @staticmethod
    def pol2cart(r: float, theta: float) -> tuple:
        """
        Function to convert polar coordinates into cartesian coordinates.\n
        :param r: distance to origin as floating point number
        :param theta: angle to x-axis as floating point number
        :return: position in x-axis and y-axis returned as tuple of two floats respectively
        """
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    def predict(self, obs_traj_pos: np.ndarray, obs_lidar, r_pos_xy: np.ndarray, r_goal_xy: np.ndarray, r_vel_state: np.ndarray) -> np.ndarray:
        """
        Main function for Dynamic Window Approach that calls other necessary functions to predict trajectories and
        choose the best proposed action.\n
        :param obs_traj_pos: position of all obstacles given as: [[x(m), y(m)], ...]
        :param r_pos_xy: current position of the robot given as: [x(m), y(m)]
        :param r_goal_xy: goal of the robot given as: [x(m), y(m)]
        :param r_vel_state: current velocity and orientation of robot given as: [v_x(m/s), v_y(m/s), theta(rad),
        v_lin(m/s), v_ang(rad/s)]
        :return: best proposed action as a numpy array
        """
        # current state of the robot given as: [x(m), y(m), theta(rad), v_lin(m/s), v_ang(rad/s)]
        r_curr_state = np.concatenate([r_pos_xy, r_vel_state[2:]])

        # best proposed action(here: u) and predicted trajectory calculated
        best_proposed_action, predicted_trajectory = self.dwa_control(r_curr_state, self.config, r_goal_xy, obs_lidar)

        return np.array([best_proposed_action])


if __name__ == '__main__':
    robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
    sim_config = ConfigNav(robot_config=robot_config)
    dwa = DWA(sim_config, -1, 30)
