import json

from numpy import array as nparray, concatenate as npconcat, cos as npcos, sin as npsin, radians as nprads
from curve import Curve, SplineType, CurveType
from utils import angle_from_slope, linspace
from waypoint import Waypoint
from robot import Robot
from typing import List
from enum import Enum


class RobotSide(Enum):
    LEFT = 1
    RIGHT = 2


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
        """
        Initializes a new Trajectory object using data defined in a given JSON file.
        :param trajectory_filename: The filename of the trajectory data file
        :param robot_filename: The filename of the robot profile data file
        :return: A new Trajectory instance, initialized with a list of Waypoints from the trajectory file and a Robot
                 from the robot file.
        """
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
        """
        Calculates the control points needed to calculate the curves.
        :return: A list of numpy vectors holding all of the needed info for the curve.
        """
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

    def curve(self, curve_type: CurveType, concat: bool = True):
        """
        Calculates the curve corresponding to the given type for the _middle_ of the robot.
        :param curve_type: The curve type wanted to calculate
        :param concat: Should concat the segments or not. Default - false
        :return: A list of numpy point vectors if concat is false. Else - one huge numpy vector
        """
        cp = self.control_points(curve_type)

        t = linspace(0, 1, samples=101)
        curves = [
            Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE).calculate(t, curve_type)
            for points in cp
        ]

        return npconcat(curves) if concat else curves

    def headings(self):
        """
        Calculates the robot's heading angles for each point in the curve (theta(t)) and calculates the derivative of
        theta(t) (theta'(t)).
        :return: A tuple consisting of the values of theta(t) and theta'(t) through the curve.
        """
        cp = self.control_points()

        t = linspace(0, 1, samples=101)
        curves = [Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE) for points in cp]

        dx, dy = npconcat([c.calculate(t, CurveType.VELOCITY) for c in curves]).T
        d2x, d2y = npconcat([c.calculate(t, CurveType.ACCELERATION) for c in curves]).T

        return angle_from_slope(dx, dy), ((d2y * dx - d2x * dy) / (dx ** 2 + dy ** 2))

    def robot_curve(self, curve_type: CurveType, side: RobotSide):
        """
        Calculates the given curve for the given side of the robot.
        :param curve_type: The type of the curve to calculate
        :param side: The side to use in the calculation
        :return: The points of the calculated curve
        """
        coeff = (self.robot.robot_info[3] / 2) * (1 if side == RobotSide.LEFT else -1)
        cp = self.control_points()

        t = linspace(0, 1, samples=101)
        curves = [
            Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE)
            for points in cp
        ]

        dx, dy = npconcat([c.calculate(t, CurveType.VELOCITY) for c in curves]).T
        d2x, d2y = npconcat([c.calculate(t, CurveType.ACCELERATION) for c in curves]).T
        theta = nprads(angle_from_slope(dx, dy))
        dtheta = (d2y * dx - d2x * dy) / (dx ** 2 + dy ** 2)

        points = npconcat([c.calculate(t, curve_type) for c in curves])
        normals = coeff * nparray([
            -npsin(theta) if curve_type == CurveType.POSITION else -npcos(theta) * dtheta,
            npcos(theta) if curve_type == CurveType.POSITION else -npsin(theta) * dtheta
        ]).T

        return points + normals
