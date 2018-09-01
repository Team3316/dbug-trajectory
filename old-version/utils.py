from typing import List, Tuple, Union, Callable, Any
from math import atan2, degrees, hypot, radians, cos, sin, fabs
from enum import Enum
import numpy as np

# Type alias
from bezier import SplineType

Point = Union[Tuple[float, float], List[int], List[float]]
PointList = List[Point]
NpCompatible = Union[float, List[float], np.float64, np.ndarray]


class Direction(Enum):
    LEFT = 1
    RIGHT = 2
    UP = 3
    DOWN = 4


class Utils(object):
    @staticmethod
    def linspace(lb: float, ub: float, samples: int) -> np.ndarray:
        sp = np.linspace(lb, ub, num=samples)
        m = np.linspace(0, 0, num=0)
        x, _ = np.meshgrid(sp, m, sparse=True)
        return x

    @staticmethod
    def angle_from_slope(dx: NpCompatible, dy: NpCompatible):
        if type(dx) != type(dy):
            raise ValueError('dx and dy should be the same type: either a float or a list of floats.')
        if type(dx) is np.float64 or type(dx) is float:
            angle = degrees(atan2(dy, dx))
            return 90 - angle if angle >= 90 else angle
        if type(dx) is np.ndarray or type(dx) is List[float]:
            return np.degrees(np.arctan2(dy, dx))

    @staticmethod
    def length_integral(t0: float, t1: float, df: Callable[[float], Any], n: int):
        """
        Calculates a segment's length using its derivative, from t0 to t1.
        The intergal is:
          I_t0^t1 sqrt((df_x(t))^2 + (df_y(t))^2)dt
        :param t0: The lower bound of the integral, 0 <= t0 <= 1
        :param t1: The upper bound of the integral, 0 <= t0 <= 1
        :param df: The function's derivative, as a function.
        :param n: The number of samples in the given derivative array
        :return: An approximated segment length, in the segment's units, using Simpson's rule.
        """
        dx = t1 - t0
        f0 = hypot(df(t0)[0], df(t0)[1])
        fn = hypot(df(t1)[0], df(t1)[1])

        fs1 = np.array([df((t0 + (dx * 2 * k)) / n) for k in range(1, int(n / 2))])
        fs2 = np.array([df((t0 + (dx * (2 * k - 1))) / n) for k in range(1, int(n / 2) + 1)])

        sum1 = 2 * np.hypot(fs1[:, 0], fs1[:, 1]).sum()
        sum2 = 4 * np.hypot(fs2[:, 0], fs2[:, 1]).sum()

        return (dx / (3 * n)) * (f0 + sum1 + sum2 + fn)

    @staticmethod
    def dts_for_heading(start_point: Point, end_point: Point, start_heading: float, end_heading: float, spline_type: SplineType):
        """
        Calculates a segment's start and end derivative vectors using it's start and end headings.
        :param start_point: The starting knot of the segment
        :param end_point: The end knot of the segment
        :param start_heading: The starting robot angle of the segment
        :param end_heading: The end robot angle of the segment
        :return: A tuple consisting of start derivative and end derivative.
        """
        start_radian = radians(start_heading)
        end_radian = radians(end_heading)

        scale = 1 if spline_type == SplineType.QUINTIC else 0.65
        start_der = start_point + np.array([-sin(start_radian), cos(start_radian)]) * scale
        end_der = end_point - np.array([-sin(end_radian), cos(end_radian)]) * scale

        scale2 = 1.5
        hdir, _ = Utils.direction(start_point, end_point)
        coeff = scale2 if hdir == Direction.LEFT else -scale2
        start_2nd_der = start_point + coeff * np.array([cos(start_radian), sin(start_radian)])
        end_2nd_der = end_point - coeff * np.array([cos(end_radian), sin(end_radian)])

        return start_der, end_der, start_2nd_der, end_2nd_der

    @staticmethod
    def direction(start_point: Point, end_point: Point):
        """
        Returns the horizontal and vertical direction of the end point from the start point.
        This is the general direction the robot will go to on the field in this segment.
        :param start_point: The starting knot of the segment
        :param end_point: The end knot of the segment
        :return: A tuple consisting of the horizontal and vertical directions.
        """
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]

        angle = Utils.angle_from_slope(dx, dy)
        is_above_start = dy > 0
        is_right = (is_above_start and 0 <= angle <= 90) or (not is_above_start and 90 <= angle <= 180)

        horizontal = Direction.RIGHT if is_right else Direction.LEFT
        vertical = Direction.UP if is_above_start else Direction.DOWN

        return horizontal, vertical
