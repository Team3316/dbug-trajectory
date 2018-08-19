from enum import Enum
import numpy as np


class CurveType(Enum):
    POSITION = 1
    VELOCITY = 2
    ACCELERATION = 3


class SplineType(Enum):
    CUBIC = 1
    QUINTIC = 2


class Bezier(object):
    # Bézier basis matrix
    B = np.array([
        [-1, 3, -3, 1],
        [3, -6, 3, 0],
        [-3, 3, 0, 0],
        [1, 0, 0, 0]
    ])

    # Bézier quintic basis matrix
    B5 = np.array([
        [-6, -3, -0.5, 6, -3, 0.5],
        [15, 8, 1.5, -15, 7, -1],
        [-10, -6, -1.5, 10, -4, 0.5],
        [0, 0, 0.5, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0]
    ])

    @classmethod
    def cubic_curve(cls, t, xs, ys, type: CurveType):
        if type == CurveType.POSITION:
            vect = np.array([t ** 3, t ** 2, t, 1]).T
        elif type == CurveType.VELOCITY:
            vect = np.array([3 * t ** 2, 2 * t, 1, 0]).T
        elif type == CurveType.ACCELERATION:
            vect = np.array([6 * t, 2, 0, 0]).T
        else:
            raise NotImplementedError('Only position, velocity and acceleration cubic curves are supported at the moment.')

        Bt = np.dot(cls.B, vect)
        return np.array([
            np.dot(xs, Bt),
            np.dot(ys, Bt)
        ])

    @classmethod
    def quintic_curve(cls, t, xs, ys, type: CurveType):
        if type == CurveType.POSITION:
            vect = np.array([t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
        elif type == CurveType.VELOCITY:
            vect = np.array([5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0])
        elif type == CurveType.ACCELERATION:
            vect = np.array([20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0])
        else:
            raise NotImplementedError('Only position, velocity and acceleration quintic curves are supported at the moment.')

        tB = np.dot(vect, cls.B5)
        return np.array([
            np.dot(tB, xs.T),
            np.dot(tB, ys.T)
        ])

    @classmethod
    def curve(cls, t, xs, ys, curve_type: CurveType, spline_type: SplineType = SplineType.CUBIC):
        if spline_type == SplineType.CUBIC:
            return cls.cubic_curve(t, xs, ys, curve_type)
        elif spline_type == SplineType.QUINTIC:
            return cls.quintic_curve(t, xs, ys, curve_type)
        else:
            raise NotImplementedError('Only cubic and quintic splines are supported at the moment.')

