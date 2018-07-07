from typing import List, Tuple, Union
from math import atan2, degrees
import numpy as np

# Type alias
Point = Tuple[float, float]
PointList = List[Point]
NpCompatible = Union[float, List[float], np.float64, np.ndarray]

class Utils(object):
    @classmethod
    def linspace(cls, lb: float, ub: float, samples: int) -> np.ndarray:
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
