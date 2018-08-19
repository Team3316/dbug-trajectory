from typing import Dict, Union, List
from trajectory import Path
from numpy import ndarray
from math import inf
import json


class Robot(object):
    """
    This class keeps information about a given robot, in order to generate paths and trajectories based on the robot's
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
        self.paths: Dict[str, Path] = {}

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

    def time_to_max(self, i: int = 6) -> float:
        """
        This calculates the time required to get the robot to 99.75% of its free speed, based on solving the ODE
        given here: https://drive.google.com/file/d/0B_lA0xR4_viqbTJYanE4X3VnWE0/view [2] and calculating its
        time constant.
        :param i: The number to multiply the time constant in. 5 results in 99.32% of free speed, 6 (default) is 99.75%.
        :return: The time to reach 99.75% of the robot's free speed using full power, in seconds.
        """
        _, _, m, _ = self.robot_info
        vf, ts, g, r = self.chassis_info
        k1 = (2 * ts * g) / (r * m * vf)
        return i / k1

    def load_path(self, filename: str):
        """
        Loads a curve into the robot's profile.
        :param filename: The filename with the curve data.
        """
        bez = Path.from_json(filename)
        path_name = bez.info[0]
        self.paths[path_name] = bez

    def gen_path(self, name: str = None, flip: bool = False, base: float = 0) -> Union[ndarray, List[ndarray]]:
        """
        Calculates the curves using this robot profile.
        :param name: The curve's name. If it isn't specified, an array with all of the paths stored will be returned.
        :param flip: Should the curves be flipped according to the middle axis of the base width?
        :param base: The base width of which the path will be flipped.
        """
        _, _, _, w = self.robot_info
        if name is None:
            curves = []
            for k in self.paths.keys():
                p = self.paths[k]
                p.gen_constraints()
                p.gen_segments()
                curves.append(p.curve(w, flip, base))
            return curves
        else:
            p = self.paths[name]
            p.gen_constraints()
            p.gen_segments()
            return p.curve(w, flip, base)
