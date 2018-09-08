import json

from utils import clamp_to_bounds
from typing import Tuple
from math import inf


class Robot(object):
    """
    A class that keeps information about a given robot, in order to generate paths and trajectories based on the robot's
    specifications. It can also generate these paths using the Bézier-based implementation, specified in `trajectory.py`.
    IMPORTANT - This profiling is currently only suited for transmissions with one kind of motor.
    """

    def __init__(self,
                 name: str = "Base Robot",
                 year: int = 2018,
                 mass: float = 0,
                 base_width: float = 0,
                 free_speed: float = 0,
                 stall_torque: float = inf,
                 gear_ratio: float = 1,
                 wheel_radius: float = 0):
        """
        Initializes the robot profile using the given parameters.
        :param name: The name of the robot.
        :param year: The year the robot was manufactured.
        :param mass: The robot's mass, in kg.
        :param base_width: The robot's chassis base width, from the center of the left wheel to the middle of the
                           right one, in meters.
        :param free_speed: The robot's free speed, in m/s. Can be calculated using JVN's calculator
        :param stall_torque: The robot's motors stall torque, in N * m. Given in the motor's specification
        :param gear_ratio: The transmissions' gear ratio.
        :param wheel_radius: The robot wheels' radius, in m.
        """
        self.robot_info = (name, year, mass, base_width)
        self.chassis_info = (free_speed, stall_torque, gear_ratio, wheel_radius)

    @classmethod
    def from_json(cls, filename: str = 'robot.json'):
        """
        Creates a new robot profile using a given JSON file.
        :param filename: The filename of the JSON file
        :return: A new instance of Robot
        """
        robot = json.loads(open(filename, 'r').read())
        return cls(
            name=robot['name'],
            year=robot['year'],
            mass=robot['mass'],
            base_width=robot['base-width'],
            free_speed=robot['free-speed'],
            stall_torque=robot['stall-torque'],
            gear_ratio=robot['gear-ratio'],
            wheel_radius=robot['wheel-radius']
        )

    def time_to_max(self, i: int = 4) -> float:
        """
        This calculates the time required to get the robot to 99.75% of its free speed, based on solving the ODE
        given here: https://drive.google.com/file/d/0B_lA0xR4_viqbTJYanE4X3VnWE0/view [2] and calculating its
        time constant.
        :param i: The number to multiply the time constant in. 5 results in 99.32% of free speed, 4 (default) is 98.17%.
        :return: The time to reach 98.17% of the robot's free speed using full power, in seconds.
        """
        _, _, m, _ = self.robot_info
        vf, ts, g, r = self.chassis_info
        k1 = (2 * ts * g) / (r * m * vf)
        return i / k1

    def max_acceleration(self) -> float:
        """
        Calculates the maximum acceleration of the robot using Newton's 2nd law.
        F_max = ma ==> a_max = F_max / m
        :return: The maximum acceleration
        """
        _, _, m, _ = self.robot_info
        _, ts, g, r = self.chassis_info
        return (ts * g) / (r * m)

    def rotational_inertia(self, linear_velocity: float, angular_velocity: float) -> float:
        """
        Calculates the robot's rotational inertia: I = mr^2. In order to calculate the current robot's turning radius,
        the linear and angular velocities are needed, since v = rw -> r = v / w. So the rotational inertia at time t is:
        I(t) = m * (v(t) / w(t))^2, where v is the linear velocity and w is the angular velocity.
        :param linear_velocity: The current linear velocity
        :param angular_velocity: The current angular velocity
        :return: The current rotational inertia of the robot.
        """
        _, _, m, _ = self.robot_info
        r = linear_velocity / angular_velocity

        return m * (r ** 2)

    def inverse_kinematics(self, left_velocity: float, right_velocity: float) -> Tuple[float, float]:
        """
        Solve the forward kinematics for the current robot (aka for the middle of the robot). The middle velocity is
        the average between the left and right velocities, and the angular velocity is calculates using the equation
        given here: http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf [3]
        :param left_velocity: The velocity of the left side of the robot
        :param right_velocity: The velocity of the right side of the robot
        :return: A tuple: (linear_velocity, angular_velocity)
        """
        _, _, _, base_width = self.robot_info
        linear = (left_velocity + right_velocity) / 2
        angular = (right_velocity - left_velocity) / base_width

        return linear, angular

    def forward_kinematics(self, linear_velocity: float, angular_velocity: float):
        """
        Solve the inverse kinematics for the current robot, according to the inverse kinematic equations given here:
        http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf [1, 2]
        :param linear_velocity: The robot's middle linear velocity
        :param angular_velocity: The robot's middle angular velocity
        :return: A tuple: (left_velocity, right_velocity)
        """
        _, _, _, base_width = self.robot_info
        free_speed, _, _, wheel_radius = self.chassis_info

        w = angular_velocity * (base_width / 2)

        left_velocity = clamp_to_bounds(-free_speed, free_speed, (linear_velocity - w))
        right_velocity = clamp_to_bounds(-free_speed, free_speed, (linear_velocity + w))

        return left_velocity, right_velocity
