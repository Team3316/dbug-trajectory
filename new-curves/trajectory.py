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

    def control_points(self, curve_type: CurveType = CurveType.POSITION):
        lw = len(self.waypoints)
        control_points = []
        for i in range(lw - 1):
            p0 = self.waypoints[i]
            p1 = self.waypoints[i + 1]
            dist = p0.distance_to(p1)
            is_not_position = curve_type != CurveType.POSITION

            control_points.append(
                nparray([
                    p0.point,
                    p0.first_derivative(scale=dist * 1, velocity=is_not_position),
                    p0.second_derivative(velocity=is_not_position),
                    p1.point,
                    p1.first_derivative(scale=dist * 1, velocity=is_not_position),
                    p1.second_derivative(velocity=is_not_position)
                ])
            )

        return control_points

    def curve(self, curve_type: CurveType, concat: bool = True):
        cp = self.control_points(curve_type)

        t = linspace(0, 1, samples=101)
        curves = [
            Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE).calculate(t, curve_type)
            for points in cp
        ]

        return npconcat(curves) if concat else curves

    def headings(self):
        cp = self.control_points()

        t = linspace(0, 1, samples=101)
        dx, dy = npconcat([
            Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE).calculate(t, CurveType.VELOCITY)
            for points in cp
        ]).T

        return angle_from_slope(dx, dy)

    def robot_curve(self, curve_type: CurveType, side: RobotSide):
        coeff = 1 if side == RobotSide.LEFT else -1
        cp = self.control_points()

        t = linspace(0, 1, samples=101)
        curves = [
            Curve(control_points=points, spline_type=SplineType.QUINTIC_HERMITE)
            for points in cp
        ]

        dx, dy = npconcat([c.calculate(t, CurveType.VELOCITY) for c in curves]).T
        headings = angle_from_slope(dx, dy)

        points = npconcat([c.calculate(t, curve_type) for c in curves])

        return points + nparray([
            coeff * -npsin(nprads(headings)).T,
            coeff * npcos(nprads(headings)).T
        ]).T