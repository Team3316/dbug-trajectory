import unittest

from curve import Curve, SplineType, CurveType
from numpy import array as nparray
from math import cos, sin, radians


# TODO - Add Bézier testing, isn't used so not tested
class CurveTests(unittest.TestCase):
    def setUp(self):
        self.pos = [
            [0, 0],
            [3, 4]
        ]

        self.vel = [
            [cos(radians(90)), sin(radians(90))],
            [cos(radians(0)), sin(radians(0))]
        ]

        self.acc = [
            [-sin(radians(90)), cos(radians(90))],
            [-sin(radians(0)), cos(radians(0))]
        ]

        quintic_data = nparray([
            self.pos[0], self.vel[0], self.acc[0],
            self.pos[1], self.vel[1], self.acc[1]
        ])
        self.quintic_hermite = Curve(SplineType.QUINTIC_HERMITE, quintic_data)

        cubic_data = nparray([
            self.pos[0], self.pos[1],
            self.vel[0], self.vel[1]
        ])
        self.cubic_hermite = Curve(SplineType.CUBIC_HERMITE, cubic_data)

    def test_quintic_hermite(self):
        curve = self.quintic_hermite

        p0 = curve.calculate(0, CurveType.POSITION)[0].tolist()
        self.assertEqual(p0, self.pos[0])

        p1 = curve.calculate(1, CurveType.POSITION)[0].tolist()
        self.assertEqual(p1, self.pos[1])

        v0 = curve.calculate(0, CurveType.VELOCITY)[0].tolist()
        self.assertEqual(v0, self.vel[0])

        v1 = curve.calculate(1, CurveType.VELOCITY)[0].tolist()
        self.assertEqual(v1, self.vel[1])

        a0 = curve.calculate(0, CurveType.ACCELERATION)[0].tolist()
        self.assertEqual(a0, self.acc[0])

        a1 = curve.calculate(1, CurveType.ACCELERATION)[0].tolist()
        self.assertEqual(a1, self.acc[1])

    def test_cubic_hermite(self):
        curve = self.cubic_hermite

        p0 = curve.calculate(0, CurveType.POSITION)[0].tolist()
        self.assertEqual(p0, self.pos[0])

        p1 = curve.calculate(1, CurveType.POSITION)[0].tolist()
        self.assertEqual(p1, self.pos[1])

        v0 = curve.calculate(0, CurveType.VELOCITY)[0].tolist()
        self.assertEqual(v0, self.vel[0])

        v1 = curve.calculate(1, CurveType.VELOCITY)[0].tolist()
        self.assertEqual(v1, self.vel[1])
