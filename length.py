from segment import Segment
from math import hypot
import numpy as np


class LengthIntegral(object):
    """
    This class is an implementation of a length integral for a 2d parameterized curve: [x(t), y(t)] between the
    points t0 and t1. It is known that the length from t0 to t (t0 <= t <= t1) is given using
        L(t) = integral(t0, t, sqrt(x'(z) ** 2 + y'(z) ** 2)dz)
    which is an integral between t0 to t. Therefore, we can use a Taylor expansion of order 3 around t0 to compute the
    integral easily and not using numerical integration methods. All of the numbering in this class start from one since
    the zero-th coefficient of the polynomial, L(t0), equals to 0. The coefficients can be obtained by differentiating
    the function n times, as per the definition of a Taylor polynomial.
    """

    # The length integral approximation requires more precision than generating a curve.
    NUM_OF_SAMPLES = 75 * 1000

    def __init__(self, segment: Segment):
        """
        Initializes the length integral approximation with the given parameters.
        :param segment: The Bézier curve segment to calculate the integral of
        """
        self.segment = segment

    def compute(self, t: float):
        """
        Computes the integral's approximation using trapezoidal approximation at a point t.
        :param t: The point to approximate around, 0 <= t <= 1
        :return: The approximated value of the integral
        """
        n = LengthIntegral.NUM_OF_SAMPLES
        _, dv = self.segment.curve(t, n)
        ndv = np.array(dv)
        midsum = np.sqrt(np.power(ndv[1:n, 0], 2) + np.power(ndv[1:n, 1], 2)).sum()

        d0 = hypot(ndv[0, 0], ndv[0, 1])
        dn = hypot(ndv[-1, 0], ndv[-1, 1])

        simpint = (t / n) * (d0 / 2 + midsum + dn / 2)
        return round(simpint, 4)
