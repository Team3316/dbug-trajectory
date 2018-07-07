from utils import Utils, PointList
from segment import Segment
from csv import DictWriter
from typing import List
import numpy as np


class Bezier(object):
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
        self.num_of_segments = len(pts) - 1

        self.complete_derivatives: np.ndarray = None
        self.curve_segments: List[Segment] = None
        self.curve_pos: np.ndarray = None
        self.curve_vel: np.ndarray = None
        self.curve_heading: np.ndarray = None

    def flip(self, base_width):
        """
        Flip a given Bézier curve.
        :return: The flipped curve coordinates
        """

        if self.curve_pos is None:
            raise NotImplementedError('A curve is needed for it to be flipped.')
        M = np.array([
            [-1, 0],
            [0, 1]
        ])
        C = np.array([
            base_width * np.ones(Segment.NUM_OF_SAMPLES * self.num_of_segments),
            np.zeros(Segment.NUM_OF_SAMPLES * self.num_of_segments)
        ])
        self.curve_pos = (np.dot(M, self.curve_pos.T) + C).T
        self.curve_heading = 180 - self.curve_heading
        return self.curve_pos

    def gen_constraints(self):
        """
        Makes an array of derivative information for each knot, giving the "Adobe handles effect" for the point planning.
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

    def gen_segments(self):
        if self.complete_derivatives is None:
            raise NotImplementedError('Complete derivative information is required for generating control points.')

        lp = len(self.pts)

        # Make an array of 5-point arrays: [p[i], d[i], d[i + 1], p[i + 1], tvector],
        # where tvector is the time vector of each point: [times[k], times[k + 1]].
        self.curve_segments = [
            Segment(
                start_point=self.pts[k],
                end_point=self.pts[k + 1],
                start_der=self.complete_derivatives[2 * k],
                end_der=self.complete_derivatives[2 * k + 1],
                start_time=self.times[k],
                end_time=self.times[k + 1]
            )
            for k in range(lp - 1)
        ]

    def curve(self, flip: bool = False, basewidth: float = None):
        """
        Calculates the position, velocity and heading information for each point in the path.
        :return: The curve position points run using np.linspace(0, 1, num=100). array<vec2>
        """
        if self.curve_segments is None:
            raise NotImplementedError('Segment array is required for generating the curve points.')

        curves = [seg.curve(1) for seg in self.curve_segments]
        self.curve_pos = np.concatenate([pos for (pos, vel) in curves])
        self.curve_vel = np.concatenate([vel for (pos, vel) in curves])

        t = Utils.linspace(0, 1, samples=Segment.NUM_OF_SAMPLES)
        self.curve_heading = np.concatenate([seg.heading(t).tolist()[0] for seg in self.curve_segments])

        if flip:
            self.flip(basewidth)

        return self.curve_pos

    def write_to_file(self, filename: str = 'curveinfo.csv'):
        """
        Writes the curve points to a CSV file.
        :param filename: The wanted filename for the file. Default - curveinfo.csv
        """
        with open(filename, 'w', newline='') as csvfile:
            fields = ['time', 'x', 'y', 'dx', 'dy']
            writer = DictWriter(csvfile, fieldnames=fields)
            writer.writeheader()
            for i in range(self.num_of_segments):
                seg = self.curve_segments[i]
                t0, t1 = seg.times
                t = Utils.linspace(t0, t1, samples=Segment.NUM_OF_SAMPLES)[0]
                rangestart = (0 if i == 0 else 1)
                writer.writerows([
                    {
                        'time': t[k],
                        'x': self.curve_pos[100 * i + k, 0],
                        'y': self.curve_pos[100 * i + k, 1],
                        'dx': self.curve_vel[100 * i + k, 0],
                        'dy': self.curve_vel[100 * i + k, 1]
                    }
                    for k in range(rangestart, Segment.NUM_OF_SAMPLES)
                ])

