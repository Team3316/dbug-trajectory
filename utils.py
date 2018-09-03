import numpy as np

from typing import Callable, Any, Union, List, Tuple
from math import degrees, atan2, hypot

# Type Aliases
Point = Union[Tuple[float, float], List[int], List[float]]
PointList = List[Point]
NpCompatible = Union[float, List[float], np.float64, np.ndarray]


def linspace(lb: float, ub: float, samples: int) -> np.ndarray:
    sp = np.linspace(lb, ub, num=samples)
    m = np.linspace(0, 0, num=0)
    x, _ = np.meshgrid(sp, m, sparse=True)
    return x


def angle_from_slope(dx: NpCompatible, dy: NpCompatible):
    if type(dx) != type(dy):
        raise ValueError('dx and dy should be the same type: either a float or a list of floats.')
    if type(dx) is np.float64 or type(dx) is float:
        angle = degrees(atan2(dy, dx))
        return 90 - angle if angle >= 90 else angle
    if type(dx) is np.ndarray or type(dx) is List[float]:
        return np.degrees(np.arctan2(dy, dx))


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


def clamp_to_bounds(lb: float, ub: float, val: float):
    """
    Clamps the given value to the given bounds.
    :param lb: The lower bound of the clamp
    :param ub: The upper bound of the clamp
    :param val: The value to clamp
    :return: The clamped value
    """
    return max(min(val, ub), lb)


def vectorify(t: NpCompatible) -> np.ndarray:
    if type(t) is float or type(t) is int or type(t) is np.float64:
        return np.array([[t]])
    elif type(t) is np.ndarray:
        return t
    else:
        return np.array(t)
