from utils import Utils, Point, NpCompatible
from typing import Union
from math import sin, cos, radians
import numpy as np


class Segment(object):
    """
    A class corresponding for making a single cubic Bézier curve segment.
    """

    # Number of samples for every picewise Bézier curve
    NUM_OF_SAMPLES = 100

    # Number of samples for length integral calculation
    L_NUM_OF_SAMPLES = 600

    # Bezier basis matrix
    B = np.array([
        [-1, 3, -3, 1],
        [3, -6, 3, 0],
        [-3, 3, 0, 0],
        [1, 0, 0, 0]
    ])

    def __init__(self,
                 start_point: Point = [0, 0],
                 end_point: Point = [0, 0],
                 start_der: Point = [0, 0],
                 end_der: Point = [0, 0],
                 start_time: float = 0,
                 end_time: float = 0,
                 origin: Point = [0, 0]):
        """
        Initializes a new cubic Bézier segment with the given paramters.
        :param start_point: The starting point of the segment.
        :param end_point: The goal point of the segment.
        :param start_der: The derivative vector corresponding to the start point.
        :param end_der: The derivative vector corresponding to the goal point.
        :param start_time: The starting time of the segment.
        :param end_time: The end time of the segment.
        :param origin: A custom origin point for the path th begin from. Default - (0, 0).
        """
        self.pts = [start_point + origin, end_point + origin]
        self.dts = [start_der + origin, end_der + origin]
        self.times = [start_time, end_time]

    @classmethod
    def apply_to_curve(cls, vect, xs, ys):
        Bt = np.dot(cls.B, vect)
        return np.array([
            np.dot(xs, Bt),
            np.dot(ys, Bt)
        ])

    def __position(self, t, xs, ys):
        """
        Returns the Bézier position polynomial result for t through the points specified using xs and ys.
        """
        vect = np.array([t ** 3, t ** 2, t, 1]).T
        return Segment.apply_to_curve(vect, xs, ys)

    def __velocity(self, t, xs, ys):
        """
        Returns the Bézier velocity (1st derivative) polynomial result for t through the points specified using xs and ys.
        """
        vect = np.array([3 * t ** 2, 2 * t, 1, 0]).T
        return Segment.apply_to_curve(vect, xs, ys)

    def __acceleration(self, t, xs, ys):
        """
        Returns the Bézier acceleration (2nd derivative) polynomial result for t through the points specified using xs and ys.
        """
        vect = np.array([6 * t, 2, 0, 0]).T
        return Segment.apply_to_curve(vect, xs, ys)

    def control_points(self):
        """
        Calculates the control vector needed for the curve generation.
        """
        return np.array([
            self.pts[0],
            self.dts[0],
            self.dts[1],
            self.pts[1],
            self.times
        ])

    def length(self, t: float = 1):
        """
        Computes the segment's length integral approximation using Simpson's rule at a point t.
        :param t: The point to approximate around, 0 <= t <= 1
        :return: The approximated value of the integral
        """
        n = Segment.L_NUM_OF_SAMPLES
        cp = self.control_points()
        xcoords = cp[0:4, 0]
        ycoords = cp[0:4, 1]
        return Utils.length_integral(t, lambda x: self.__velocity(x, xcoords, ycoords), n)

    def robot_lengths(self, t: float, basewidth: float):
        """
        Computes the segment's length integral approximation for each side of the robot using Simpson's rule at a point t.
        :param t: The point to approximate around, 0 <= t <= 1
        :param basewidth: The width of the robot's chassis, measured from the center of the wheels.
        :return: The approximated value of the integral for the left and right sides of the robot
        """
        n = Segment.L_NUM_OF_SAMPLES
        cp = self.control_points()
        xcoords = cp[0:4, 0]
        ycoords = cp[0:4, 1]

        dcos = lambda x: -(basewidth / 2) * cos(radians(self.heading(x)))
        dsin = lambda x: -(basewidth / 2) * sin(radians(self.heading(x)))
        dnormal = lambda x: np.array([dcos(x) * self.__dheading(x), dsin(x) * self.__dheading(x)])

        dl = lambda x: self.__velocity(x, xcoords, ycoords) + dnormal(x)
        dr = lambda x: self.__velocity(x, xcoords, ycoords) - dnormal(x)

        leftdist = Utils.length_integral(t, dl, n)
        rightdist = Utils.length_integral(t, dr, n)
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
        xcoords = cp[0:4, 0]
        ycoords = cp[0:4, 1]

        # Run the functions!
        x = Utils.linspace(0, t, samples=n)
        respos = self.__position(x, xcoords, ycoords).T
        pos = respos[:, 0].tolist()

        resvel = self.__velocity(x, xcoords, ycoords).T
        vel = resvel[:, 0].tolist()

        resacc = self.__acceleration(x, xcoords, ycoords).T
        acc = resacc[:, 0].tolist()

        return pos, vel, acc

    def robot_curve(self, t: float, basewidth: float, n: int = NUM_OF_SAMPLES):
        """
        Calculates the position curve for the robot's left and right sides between 0 and t.
        :param t: The end point of the segment.
        :param basewidth: The width of the robot's chassis, measured from the center of the wheels.
        :param n: The number of samples to create of the curve.
        :return: Left and right position values for the interval [0, t].
        """
        if t < 0 or t > 1:
            raise AssertionError('t is not in range, should be between 0 and 1.')

        cp = self.control_points()
        xcoords = cp[0:4, 0]
        ycoords = cp[0:4, 1]

        x = Utils.linspace(0, t, samples=n)
        pos = self.__position(x, xcoords, ycoords).T[:, 0]

        nangles = np.radians(90 + self.heading(x))  # The angles required for the normal vectors
        normals = np.array([np.cos(nangles), np.sin(nangles)])[:, 0].T

        pleft = pos + (basewidth / 2) * normals
        pright = pos - (basewidth / 2) * normals

        return pleft, pright

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
        xcoords = cp[0:4, 0]
        ycoords = cp[0:4, 1]
        dx, dy = self.__velocity(t, xcoords, ycoords)
        return Utils.angle_from_slope(dx, dy)

    def __dheading(self, t: NpCompatible):
        cp = self.control_points()
        xcoords = cp[0:4, 0]
        ycoords = cp[0:4, 1]
        dx, dy = self.__velocity(t, xcoords, ycoords)
        d2x, d2y = self.__acceleration(t, xcoords, ycoords)
        return (d2y * dx - d2x * dy) / (dx ** 2 + dy ** 2)

