from math import atan2, degrees, hypot
from typing import List, Tuple, Union
import numpy as np

# Type alias
Point = Union[Tuple[float, float], List[int], List[float]]
PointList = List[Point]
NpCompatible = Union[float, List[float], np.float64, np.ndarray]

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
            return np.arctan2(dy, dx) * 180 / np.pi

    @staticmethod
    def length_integral(t: float, f: np.ndarray, n: int):
        """
        Calculates a segment's length using its derivative values from 0 to t.
        :param t: The point to approximate around, 0 <= t <= 1
        :param f: The function's derivative values in the segment
        :param n: The number of samples in the given derivative array
        :return: An approximated segment length, in the segment's units.
        """
        midsum = np.sqrt(np.power(f[1:n, 0], 2) + np.power(f[1:n, 1], 2)).sum()

        d0 = hypot(f[0, 0], f[0, 1])
        dn = hypot(f[-1, 0], f[-1, 1])

        simpint = (t / n) * (d0 / 2 + midsum + dn / 2)
        return round(simpint, 4)
