"""
    Configurations File
"""
from enum import Enum


# simple class for robot types
class RobotType(Enum):
    circle = 0
    rectangle = 1


class ConfigRobot:
    """
    Class to configure the properties of the robot for DWA. The properties are contained inside a dictionary.
    """
    def __init__(self, robot_model: str = "locobot", collision_dist: float = 0.35, robot_params: dict = None):
        """
        Simple constructor to set the properties of the robot.\n
        :param robot_model: name of the robot model
        :param collision_dist: baseline distance to obstacles that the robot should always avoid. given in meters
        :param robot_params: dictionary containing robot's capabilities such as max/min-speed and max-accel for angular and linear velocity
        """
        if robot_model == "locobot":
            self.robot_params = {'max_linear_vel': .3,  # [m/s]
                                 'min_linear_vel': -.1,  # [m/s]
                                 'max_linear_acc': .5,  # [m/ss]
                                 'max_angular_vel': 1.0,  # [rad/s]
                                 'max_angular_acc': 3.2,  # [rad/ss] / collision_dist: [m]
                                 'collision_dist': collision_dist} if robot_params is None else robot_params
        else:
            raise ValueError("There is no other robot we are working with, just use locobot dummy")


class ConfigNav:
    """
    Class to configure the navigation of the robot. Contains cost gains and time tick for motion prediction. Uses
    configuration from ConfigRobot class.
    """
    def __init__(self, robot_config: ConfigRobot):
        """
        Simple constructor to initialize the configuration of navigation.\n
        :param robot_config: ConfigRobot object containing parameters of
        """
        # robot parameters
        self.robot_model = robot_config.robot_params
        self.max_speed = self.robot_model['max_linear_vel']  # [m/s]
        self.min_speed = self.robot_model['min_linear_vel']  # [m/s]
        self.max_accel = self.robot_model['max_linear_acc']  # [m/ss]
        self.max_yaw_rate = self.robot_model['max_angular_vel']  # [rad/s]
        self.max_delta_yaw_rate = self.robot_model['max_angular_acc']  # [rad/ss]

        # the radius, in which we consider obstacles
        self.inner_proximity_radius = 2  # [m]

        # robot attributes
        self.robot_type = RobotType.circle
        self.robot_radius = 0.35  # [m] for collision check

        ### navigation params

        # sampling resolutions (to be used when choosing & evaluating trajectories within the Dynamic Window)
        self.v_resolution = 0.025  # [m/s]
        self.yaw_rate_resolution = 0.025  # sim_granularity like in move_base
        self.dt = 0.4  # [s] Time tick for motion prediction
        self.pred_time_steps = 2

        # max. allowed time for motion prediction (predict_time / dt -> no. predictions per sampled (v_lin, v_ang) pair)
        self.predict_time = self.pred_time_steps * 0.4  # 1.7s for move_base we tested 4 * 0.4, but best results with 2 * 0.4
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stuck

        ### Prediction model parameters
        self.pred_time_steps = 2

        ### weights for cost calculation

        # weight for the angle difference between the current "predicted" trajectory's angle and the optimal angle to
        # reach the goal
        self.to_goal_cost_gain = 0.5

        # weight for the difference between the linear velocity of the current "predicted" trajectory and the max
        # allowed linear velocity
        self.speed_cost_gain = 5

        # weight for (1 / dist. to closest obstacle)
        self.obstacle_cost_gain = 2
