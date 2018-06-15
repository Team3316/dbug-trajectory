from typing import List, Tuple
from csv import DictWriter
from utils import Utils
import numpy as np

# Type alias
PointList = List[Tuple[float, float]]

class Bezier(object):
    # Number of samples for every picewise Bézier curve
    NUM_OF_SAMPLES = 100

    # Bezier basis matrix
    B = np.array([
        [-1, 3, -3, 1],
        [3, -6, 3, 0],
        [-3, 3, 0, 0],
        [1, 0, 0, 0]
    ])

    def __init__(self, pts: PointList, dts: PointList, times: List[float]):
        """
        :param pts: Knot array. array<vec2>
        :param dts: Array of derivative information for each knot, assuming
                    that dts[i] has the derivative information for pts[i],
                    and that dts[i] is "to the left" of pts[i] -- which means
                    that if dts[i] = (dx, dy) and pts[i] = (x, y) then dx =< x. array<vec2>
        :param times: Array of time information for each knot, meaning that
                      the robot should be in pts[i] at time times[i]. array<float>
        """
        self.pts = np.array(pts)
        self.dts = np.array(dts)
        self.times = np.array(times)

        self.complete_derivatives: np.ndarray = None
        self.curve_info: np.ndarray = None
        self.curve_points: np.ndarray = None

    def polynomial(self, t, xs, ys):
        """
        Returns the Bézier polynomial result for t through the points specified using xs and ys.
        """
        vect = np.array([t ** 3, t ** 2, t, 1]).T
        Bt = np.dot(Bezier.B, vect)
        return np.array([
            np.dot(xs, Bt),
            np.dot(ys, Bt)
        ])

    def flip(self, base_width):
        """
        Flip a given Bézier curve.
        :return: The flipped curve coordinates
        """
        if self.curve_points is None:
            raise NotImplementedError('A curve is needed for it to be flipped.')
        M = np.array([
            [-1, 0],
            [0, 1]
        ])
        C = np.array([
            base_width * np.ones(Bezier.NUM_OF_SAMPLES),
            np.zeros(Bezier.NUM_OF_SAMPLES)
        ])
        self.curve_points = np.array([np.dot(M, a) + C for a in self.curve_points])
        return self.curve_points

    def gen_derivatives(self):
        """
        Makes an array of derivative information for each knot, giving the "Adobe handles effect" for the point planning.
        :return:
        """
        ld = len(self.dts)
        derivatives = []
        for (i, dt) in enumerate(self.dts):
            if i == 0 or i == ld - 1:
                derivatives.append(dt)
            else:
                derivatives.append(dt)
                derivatives.append(2 * self.pts[i] - dt)
        self.complete_derivatives = derivatives

    def gen_control_points(self):
        if self.complete_derivatives is None:
            raise NotImplementedError('Complete derivative information is required for generating control points.')

        lp = len(self.pts)

        # Make an array of 5-point arrays: [p[i], d[i], d[i + 1], p[i + 1], tvector],
        # where tvector is the time vector of each point: [times[k], times[k + 1]].
        self.curve_info = np.array([
            [
                self.pts[k],
                self.complete_derivatives[2 * k],
                self.complete_derivatives[2 * k + 1],
                self.pts[k + 1],
                [
                    self.times[k],
                    self.times[k + 1]
                ]
            ]
            for k in range(lp - 1)
        ])

    def curve(self, flip: bool = False, basewidth: float = None):
        """
        Returns a piecewise Bézier curve, using derivative information for each knot.
        :return: The curve points run using np.linspace(0, 1, num=1000). array<vec2>
        """
        if self.curve_info is None:
            raise NotImplementedError('Control point array is required for generating the curve points.')

        control = self.curve_info

        # Get the x and y coordinates
        xcoords = control[:, 0:4, 0]
        ycoords = control[:, 0:4, 1]

        # Run the function!
        x = Utils.linspace(0, 1, samples=Bezier.NUM_OF_SAMPLES)
        res = self.polynomial(x, xcoords, ycoords).T
        res = np.array([[res[c, 0][0].T, res[c, 1][0].T] for c in range(len(res))])
        self.curve_points = res

        if flip:
            self.flip(basewidth)

        return self.curve_points

    def write_to_file(self, filename: str = 'curveinfo.csv'):
        with open(filename, 'w', newline='') as csvfile:
            fields = ['time', 'xpos', 'ypos']
            writer = DictWriter(csvfile, fieldnames=fields)
            writer.writeheader()
            for i in range(len(self.curve_points)):
                t0, t1 = self.curve_info[i, 4]
                t = Utils.linspace(t0, t1, samples=Bezier.NUM_OF_SAMPLES)[0]
                writer.writerows([{'time': t[k], 'xpos': self.curve_points[i, 0, k], 'ypos': self.curve_points[i, 1, k]} for k in range(Bezier.NUM_OF_SAMPLES) ])

