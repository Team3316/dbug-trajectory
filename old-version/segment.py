from bezier import Bezier, CurveType, SplineType
from utils import Utils, Point, NpCompatible
from typing import Union
from math import sin, cos, radians
import numpy as np


class Segment(object):
    """
    A class corresponding for making a single cubic / quintic Bézier curve segment.
    """

    # Number of samples for every picewise Bézier curve
    NUM_OF_SAMPLES = 101

    # Number of samples for full segment length integral calculation
    L_NUM_OF_SAMPLES = 600

    def __init__(self,
                 start_point: Point = [0, 0],
                 end_point: Point = [0, 0],
                 start_der: Point = [0, 0],
                 end_der: Point = [0, 0],
                 start_second_der: Point = [0, 0],
                 end_second_der: Point = [0, 0],
                 start_time: float = 0,
                 end_time: float = 0,
                 origin: Point = [0, 0],
                 spline_type: SplineType = SplineType.CUBIC):
        """
        Initializes a new cubic Bézier segment with the given paramters.
        :param start_point: The starting point of the segment.
        :param end_point: The goal point of the segment.
        :param start_der: The derivative vector corresponding to the start point.
        :param end_der: The derivative vector corresponding to the goal point.
        :param start_second_der: The second derivative vector corresponding to the starting point.
        :param end_second_der: The second derivative vector corresponding to the goal point.
        :param start_time: The starting time of the segment.
        :param end_time: The end time of the segment.
        :param origin: A custom origin point for the path th begin from. Default - (0, 0).
        """
        self.pts = [start_point + origin, end_point + origin]
        self.dts = [start_der + origin, end_der + origin] if spline_type != SplineType.QUINTIC else [start_der, end_der]
        self.d2ts = [start_second_der, end_second_der]
        self.times = [start_time, end_time]
        self.spline_type = spline_type

    def control_points(self):
        """
        Calculates the control vector needed for the curve generation.
        """
        p0, p1 = self.pts
        dp0, dp1 = self.dts
        ddp0, ddp1 = self.d2ts

        cubic_cp = [p0, dp0, dp1, p1, self.times]
        quintic_cp = [p0, dp0, ddp0, p1, dp1, ddp1, self.times]

        cp = cubic_cp if self.spline_type != SplineType.QUINTIC else quintic_cp
        return np.array(cp)

    def length(self, t0: float = 0, t1: float = 1):
        """
        Computes the segment's length integral approximation using Simpson's rule at a point t.
        :param t0: The lower bound of the integral, 0 <= t0 <= 1
        :param t1: The upper bound of the integral, 0 <= t0 <= 1
        :return: The approximated value of the integral
        """
        n = Segment.L_NUM_OF_SAMPLES
        cp = self.control_points()
        xcoords = cp[0:-1, 0]
        ycoords = cp[0:-1, 1]
        curve = lambda x: Bezier.curve(x, xcoords, ycoords, CurveType.VELOCITY, self.spline_type)

        return Utils.length_integral(t0, t1, curve, n)

    def robot_lengths(self, basewidth: float, t0: float = 0, t1: float = 1, n: int = L_NUM_OF_SAMPLES):
        """
        Computes the segment's length integral approximation for each side of the robot using Simpson's rule at a point t.
        :param t0: The lower bound of the integral, 0 <= t0 <= 1
        :param t1: The upper bound of the integral, 0 <= t0 <= 1
        :param basewidth: The width of the robot's chassis, measured from the center of the wheels.
        :param n: The number of samples to create of the interval's partition.
        :return: The approximated value of the integral for the left and right sides of the robot
        """
        cp = self.control_points()
        xcoords = cp[0:-1, 0]
        ycoords = cp[0:-1, 1]

        dcos = lambda x: -(basewidth / 2) * cos(radians(self.heading(x)))
        dsin = lambda x: -(basewidth / 2) * sin(radians(self.heading(x)))
        dnormal = lambda x: np.array([dcos(x) * self.__dheading(x), dsin(x) * self.__dheading(x)])

        vel = lambda x: Bezier.curve(x, xcoords, ycoords, CurveType.VELOCITY, self.spline_type)
        dl = lambda x: vel(x) + dnormal(x)
        dr = lambda x: vel(x) - dnormal(x)

        leftdist = Utils.length_integral(t0, t1, dl, n)
        rightdist = Utils.length_integral(t0, t1, dr, n)
        return leftdist, rightdist

    def curve(self, t: float, n: int = NUM_OF_SAMPLES):
        """
        Calculates the position and position derivative curve between 0 and t.
        :param t: The end point of the segment.
        :param n: The number of samples to create of the curve.
        :return: Position and velocity values for the interval [0, t].
        """
        if t < 0 or t > 1:
            raise AssertionError('t is not in range, should be between 0 and 1.')

        # Get the x and y coordinates
        cp = self.control_points()
        xcoords = cp[0:-1, 0]
        ycoords = cp[0:-1, 1]

        # Run the functions!
        x = Utils.linspace(0, t, samples=n)
        respos = Bezier.curve(x, xcoords, ycoords, CurveType.POSITION, self.spline_type).T
        pos = respos[:, 0].tolist()

        resvel = Bezier.curve(x, xcoords, ycoords, CurveType.VELOCITY, self.spline_type).T
        vel = resvel[:, 0].tolist()

        resacc = Bezier.curve(x, xcoords, ycoords, CurveType.ACCELERATION, self.spline_type).T
        acc = resacc[:, 0].tolist()

        return pos, vel, acc

    def robot_curve(self, t: float, basewidth: float, n: int = NUM_OF_SAMPLES):
        """
        Calculates the position curve for the robot's left and right sides between 0 and t.
        :param t: The end point of the segment.
        :param basewidth: The width of the robot's chassis, measured from the center of the wheels.
        :param n: The number of samples to create of the curve.
        :return: Left position, right position, left velocity and right velocity values for the interval [0, t].
        """
        if t < 0 or t > 1:
            raise AssertionError('t is not in range, should be between 0 and 1.')

        cp = self.control_points()
        xcoords = cp[0:-1, 0]
        ycoords = cp[0:-1, 1]

        x = Utils.linspace(0, t, samples=n)
        pos = Bezier.curve(x, xcoords, ycoords, CurveType.POSITION, self.spline_type).T[:, 0]
        vel = Bezier.curve(x, xcoords, ycoords, CurveType.VELOCITY, self.spline_type).T[:, 0]

        nangles = np.radians(90 + self.heading(x))  # The angles required for the normal vectors
        normals = np.array([np.cos(nangles), np.sin(nangles)])[:, 0].T
        dnormals = (np.array([-np.sin(nangles), np.cos(nangles)])[:, 0] * self.__dheading(x)).T

        pleft = pos + (basewidth / 2) * normals
        pright = pos - (basewidth / 2) * normals

        resvleft = vel + (basewidth / 2) * dnormals
        resvright = vel - (basewidth / 2) * dnormals

        vleft = np.hypot(resvleft[:, 0], resvleft[:, 1])
        vright = np.hypot(resvright[:, 0], resvright[:, 1])

        return pleft, pright, vleft, vright

    def flip(self, base_width: Union[float, int]):
        """
        Flip this segment, according to a base with width base_width.
        :return: The flipped curve coordinates
        """
        lp = len(self.pts)
        M = np.array([
            [-1, 0],
            [0, 1]
        ])
        C = np.array([
            base_width * np.ones(lp),
            np.zeros(lp)
        ]).T

        npts = (np.dot(np.array(self.pts), M) + C).tolist()
        self.pts = [np.array(a) for a in npts]

        ndts = (np.dot(np.array(self.dts), M) + C).tolist()
        self.dts = [np.array(a) for a in ndts]

        return self

    def heading(self, t: NpCompatible):
        """
        Calculates the heading angles at time/s t. t can be both a singular and a plural time, in order to make the
        function simpler.
        :param t: A single point in time or a number of points in time.
        :return: The heading angle in time/s t.
        """
        cp = self.control_points()
        xcoords = cp[0:-1, 0]
        ycoords = cp[0:-1, 1]
        dx, dy = Bezier.curve(t, xcoords, ycoords, CurveType.VELOCITY, self.spline_type)
        return Utils.angle_from_slope(dx, dy)

    def __dheading(self, t: NpCompatible):
        cp = self.control_points()
        xcoords = cp[0:-1, 0]
        ycoords = cp[0:-1, 1]
        dx, dy = Bezier.curve(t, xcoords, ycoords, CurveType.VELOCITY, self.spline_type)
        d2x, d2y = Bezier.curve(t, xcoords, ycoords, CurveType.ACCELERATION, self.spline_type)
        return (d2y * dx - d2x * dy) / (dx ** 2 + dy ** 2)

