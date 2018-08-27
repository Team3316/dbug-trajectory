import numpy as np

from typing import Callable, Union
from numpy.linalg import multi_dot
from utils import Utils
from enum import Enum

TimeVariable = Union[float, np.ndarray]
TimeFunction = Callable[[TimeVariable], np.ndarray]


class SplineType(Enum):
    CUBIC_BEZIER = 1
    CUBIC_HERMITE = 2
    QUINTIC_HERMITE = 3


class CurveType(Enum):
    POSITION = 1
    VELOCITY = 2
    ACCELERATION = 3


class Curve:
    """
    A class representing a spline curve. Calculating each spline requires 3 ingredients:
    1. A time monomials vector - for example, a cubic position spline will need the following
       vector in order to compute: [t ** 3, t ** 2, t, 1], and a cubic velocity spline will use
       the position curve's vector first derivative: [3 * t ** 2, 2 * t, 1, 0].
    2. The _constant_ spline basis matrix - this depends on the specific spline type. For cubic splines the
       basis matrix will be of dimension (3, 3) and quintic splines will be of dimension (5, 5). Generally,
       an n-degree spline will use an (n, n) dimensional matrix. The supported splines' basis matrices
       are given in the class.
    3. The points to interpolate through - each spline requires has it's own formatting for this vector.
       Each and every basis matrix has the required formatting for the spline written above it.
    """

    # Bézier cubic basis matrix
    # tvec = [t ** 3, t ** 2, t, 1]
    # pvec = [p0, dp0, dp1, p1]
    B3 = np.array([
        [-1, 3, -3, 1],
        [3, -6, 3, 0],
        [-3, 3, 0, 0],
        [1, 0, 0, 0]
    ])

    # Hermite cubic basis matrix
    # tvec = [t ** 3, t ** 2, t, 1]
    # pvec = [p0, p1, dp0, dp1]
    H3 = np.array([
        [2, -2, 1, 1],
        [-3, 3, -2, -1],
        [0, 0, 1, 0],
        [1, 0, 0, 0]
    ])

    # Hermite quintic basis matrix
    # tvec = [t ** 5, t ** 4, t ** 3, t ** 2, t, 1]
    # pvec = [p0, dp0, d2p0, p1, dp1, d2p1]
    H5 = np.array([
        [-6, -3, -0.5, 6, -3, 0.5],
        [15, 8, 1.5, -15, 7, -1],
        [-10, -6, -1.5, 10, -4, 0.5],
        [0, 0, 0.5, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0]
    ])

    def __init__(self, spline_type: SplineType, control_points: np.ndarray):
        """
        Initializes a new curve.
        :param spline_type: The type of the curve's interpolation spline
        :param control_points: The control points array of the curve. Should fit the format of the given curve type
        """
        self.spline_type = spline_type
        self.control_points = control_points

    @classmethod
    def basis_matrix_for_type(cls, spline_type: SplineType):
        """
        Returns the basis matrix for the given curve interpolation type
        :param spline_type: The given curve interpolation type
        :return: A basis matrix needed for calculating the interpolated polynomial of the curve
        """
        return {
            SplineType.CUBIC_BEZIER: cls.B3,
            SplineType.CUBIC_HERMITE: cls.H3,
            SplineType.QUINTIC_HERMITE: cls.H5
        }[spline_type]

    @classmethod
    def time_vector_for_type(cls, spline_type: SplineType, curve_type: CurveType) -> TimeFunction:
        """
        Returns a function that accepts a time parameter, t, and returns the respecting polynomial basis for the give
        spline and curve type.
        :param spline_type: The spline type of the output basis polynomial, needed for the basis degree
        :param curve_type: The curve type of the output basis polynomial
        :return: A function (float | ndarray) -> ndarray
        """
        degree = 5 if spline_type == SplineType.QUINTIC_HERMITE else 3
        return lambda t: np.array({
            CurveType.POSITION: [t ** (degree - i) for i in range(degree + 1)],
            CurveType.VELOCITY: [(degree - i) * t ** (degree - i - 1) for i in range(degree)] + [0],
            CurveType.ACCELERATION: [(degree - i) * (degree - i - 1) * t ** (degree - i - 2) for i in range(degree - 1)] + [0, 0]
        }[curve_type])

    def calculate(self, t: TimeVariable, curve_type: CurveType) -> np.ndarray:
        """
        Calculates the curve at time(s)
        :param t: The time(s) to calculate the curve in
        :param curve_type: The curve type to calculate
        :return: The point(s) p(t), where p is the curve function.
        """
        M = Curve.basis_matrix_for_type(self.spline_type)
        v = Curve.time_vector_for_type(self.spline_type, curve_type)(t)[:, 0, :]

        xs = self.control_points[:, 0]
        ys = self.control_points[:, 1]

        return np.array([
            multi_dot([v.T, M, xs]),
            multi_dot([v.T, M, ys])
        ]).T

    # TODO - DELETE!!! This function is ONLY intended for testing purposes
    def desmos_points(self, curve_type: CurveType):
        x = Utils.linspace(0, 1, samples=100)
        points = ['({}, {})'.format(round(p[0], 4), round(p[1], 4)) for p in self.calculate(x, curve_type)]
        return ','.join(points)
