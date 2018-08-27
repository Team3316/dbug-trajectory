import json

from numpy import array as nparray, concatenate as npconcat
from curve import Curve, SplineType, CurveType
from waypoint import Waypoint
from robot import Robot
from typing import List
from utils import Utils


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

    @classmethod
    def from_json(cls, trajectory_filename: str, robot_filename: str):
        file = open(trajectory_filename, 'r').read()
        decoded = json.loads(file)
        waypoints = [
            Waypoint(
                point=waypoint['point'],
                angle=waypoint['heading'],
                time=waypoint['time']
            )
            for waypoint in decoded['waypoints']
        ]
        robot = Robot.from_json(robot_filename)
        return cls(waypoints, robot)

    def control_points(self):
        lw = len(self.waypoints)
        control_points = []
        for i in range(lw - 1):
            p0 = self.waypoints[i]
            p1 = self.waypoints[i + 1]
            dist = p0.distance_to(p1)

            control_points.append(
                nparray([
                    p0.point,
                    p0.first_derivative(scale=dist * 1),
                    p0.second_derivative(),
                    p1.point,
                    p1.first_derivative(scale=dist * 1),
                    p1.second_derivative()
                ])
            )

        return control_points

    def curve(self):
        cp = self.control_points()

        t = Utils.linspace(0, 1, samples=101)
        curves = [
            Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE)
            for points in cp
        ]

        return npconcat([c.calculate(t, CurveType.POSITION) for c in curves])