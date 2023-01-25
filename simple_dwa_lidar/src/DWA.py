import warnings
import math
import numpy as np
import torch
import os
from config import ConfigRobot, ConfigSim, RobotType

warnings.filterwarnings("ignore", category=DeprecationWarning)


class DWA:
    def __init__(self, sim_config: ConfigSim, num_agent: int):
        self.robot_model = sim_config.robot_model
        self.init_Flag = True
        self.num_agent = num_agent
        self.config = sim_config
        self.num_agent = num_agent

    def dwa_control(self, x, config, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x, config)

        u, trajectory = self.calc_control_and_trajectory(x, dw, config, goal, ob)
        return u, trajectory

    def motion(self, x, u, dt):
        """
        motion model
        """
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        # TODO: debug this function
        # u -> linear(x) and angular vel.(z) action / vel. -> 2 wheels needed to created curves
        # x -> current state for given time-step
        # publish u to cmd_vel
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    def calc_dynamic_window(self, x, config):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [config.min_speed, config.max_speed,
              -config.max_yaw_rate, config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - config.max_accel * config.dt,
              x[3] + config.max_accel * config.dt,
              x[4] - config.max_delta_yaw_rate * config.dt,
              x[4] + config.max_delta_yaw_rate * config.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def predict_trajectory(self, x_init, v, y, config):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            trajectory = np.vstack((trajectory, x))
            time += config.dt

        return trajectory

    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def calc_obstacle_cost(self, trajectory, ob, config):
        """
        calc obstacle cost inf: collision
        """
        
        ox = ob[:, 0]
        oy = ob[:, 1]

        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if config.robot_type == RobotType.circle:
            if np.array(r <= config.robot_radius).any():
                return float("Inf")
        else:
            print("Chose the wrong Robot Type, bruh moment")
            exit(-1)

        min_r = np.nanmin(r)

        return 1.0 / min_r  # OK

    def calc_control_and_trajectory(self, x, dw, config, goal, ob):
        """
        calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], config.v_resolution):
            for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y, config)
                # calc cost
                to_goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
                ob_cost = config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, config)
                #print("goal:" + str(to_goal_cost))
                #print("speed:" + str(speed_cost))
                #print("ob:" + str(ob_cost))
               # print("-------")
                final_cost = to_goal_cost + speed_cost + ob_cost
                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < config.robot_stuck_flag_cons \
                            and abs(x[3]) < config.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -config.max_yaw_rate  # Bug in original code? Corrected to max_yaw_rate instead
                        # max_delta_yaw_rate
        return best_u, best_trajectory

    def cart2pol(self, x, y):
        """
        Function to convert cartesian coordinates into polar coordinates.
        """
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)
        return rho, phi

    def pol2cart(self, rho, phi):
        """
        Function to convert polar coordinates into cartesian coordinates.
        """
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return x, y

    def predict(self, obs_traj_pos: np.ndarray, obs_lidar, r_pos_xy: np.ndarray, r_goal_xy: np.ndarray, r_vel_state: np.ndarray) -> np.ndarray:
        """
        # r_state -> v_x, v_y, orientation, a_x, v_orientation
        r_state = r_state
        # r_pos -> x, y
        r_state_xy = r_pos
        # r_goal
        r_goal = r_goal

        """
        obs_traj_pos = obs_lidar

        yaw = r_vel_state[2]

        # init obstacles in vicinity with dummy obstacle far away
        obs_xy_in_proximity_list = [[-1000, -1000]]

        # unit vector at origin, with the same direction as the robot
        r_norm = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float)
        for idx, ob in enumerate(obs_traj_pos[-1, :, :]):
            # proximity check
            if np.linalg.norm(r_pos_xy - ob) <= self.config.inner_proximity_radius:
                obs_xy_in_proximity_list.append(ob)

        all_obs = obs_traj_pos

        # current state of the robot given as: [x(m), y(m), theta(rad), v_lin(m/s), v_ang(rad/s)]
        r_curr_state = np.concatenate([r_pos_xy, r_vel_state[2:]])

        # best proposed action(here: u) and predicted trajectory calculated
        best_proposed_action, predicted_trajectory = self.dwa_control(r_curr_state, self.config, r_goal_xy, all_obs)

        return np.array([best_proposed_action])
        
        

if __name__ == '__main__':
    robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
    sim_config = ConfigSim(robot_config=robot_config)
    dwa = DWA(sim_config, -1, 30)
