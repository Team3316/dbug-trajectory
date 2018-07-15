from typing import List, Tuple, Union, Callable, Any
from math import atan2, degrees, hypot
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
    def length_integral(t: float, df: Callable[[float], Any], n: int):
        """
        Calculates a segment's length using its derivative.
        :param t: The point to approximate around, 0 <= t <= 1
        :param df: The function's derivative, as a function.
        :param n: The number of samples in the given derivative array
        :return: An approximated segment length, in the segment's units, using Simpson's rule.
        """
        f0 = hypot(df(0)[0], df(0)[1])
        fn = hypot(df(t)[0], df(t)[1])

        fs1 = np.array([df((t * 2 * k) / n) for k in range(1, int(n / 2))])
        fs2 = np.array([df((t * (2 * k - 1)) / n) for k in range(1, int(n / 2) + 1)])

        sum1 = 2 * np.hypot(fs1[:, 0], fs1[:, 1]).sum()
        sum2 = 4 * np.hypot(fs2[:, 0], fs2[:, 1]).sum()

        return (t / (3 * n)) * (f0 + sum1 + sum2 + fn)
