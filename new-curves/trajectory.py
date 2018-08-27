from .waypoint import Waypoint
from .robot import Robot
from typing import List


class Trajectory:
    """
    A class that represents a robot's trajectory - with abilities to generate the needed
    curves for the given robot's profile and according to the given waypoints.
    """

    def __init__(self, waypoints: List[Waypoint], robot: Robot):
        """
        Creates a new Trajectory.
        :param waypoints: The waypoints the trajectory should go through
        :param robot: The robot profile to use
        """
        self.waypoints = waypoints
        self.robot = robot

    def curve(self):
        pass