from utils import Utils, PointList
from typing import List, Tuple
from segment import Segment
from csv import DictWriter
import json
import numpy as np


class Bezier(object):
    def __init__(self,
                 pts: PointList,
                 dts: PointList,
                 times: List[float],
                 name: str = "untitled-path",
                 desc: str = "This is an untitled path."):
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

        self.info = (name, desc)

        self.origin = [0, 0]
        self.complete_derivatives: np.ndarray = None
        self.curve_segments: List[Segment] = None
        self.curve_pos: np.ndarray = None
        self.curve_vel: np.ndarray = None
        self.curve_heading: np.ndarray = None

        self.curve_robotl: np.ndarray = None
        self.curve_robotr: np.ndarray = None
        self.curve_robotdist_left: np.ndarray = None
        self.curve_robotdist_right: np.ndarray = None

    @classmethod
    def from_json(cls, filename: str):
        file = open(filename, 'r').read()
        decoded = json.loads(file)
        knots = decoded['knots']
        pts = [knot['point'] for knot in knots]
        dts = [knot['derivative'] for knot in knots]
        times = [knot['time'] for knot in knots]
        bez = cls(pts, dts, times, decoded['name'], decoded['description'])
        bez.origin = np.array(decoded['origin']) + [decoded['robot-width'] / 2, 0]
        return bez

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
                end_time=self.times[k + 1],
                origin=self.origin
            )
            for k in range(lp - 1)
        ]

    def curve(self, robot_width: float, flip: bool = False, basewidth: float = None):
        """
        Generates the robot curves, heading values and length values.
        """
        if self.curve_segments is None:
            raise NotImplementedError('Segment array is required for generating the curve points.')

        if flip:
            self.curve_segments = [seg.flip(basewidth) for seg in self.curve_segments]

        curves = [seg.curve(1) for seg in self.curve_segments]
        self.curve_pos = np.concatenate([pos for (pos, vel, acc) in curves])
        self.curve_vel = np.concatenate([vel for (pos, vel, acc) in curves])

        t = Utils.linspace(0, 1, samples=Segment.NUM_OF_SAMPLES)
        self.curve_heading = np.concatenate([seg.heading(t).tolist()[0] for seg in self.curve_segments])

        robot_curves = [seg.robot_curve(1, robot_width) for seg in self.curve_segments]
        self.curve_robotl = np.concatenate([tup[0] for tup in robot_curves])
        self.curve_robotr = np.concatenate([tup[1] for tup in robot_curves])

        seg_lengths = [
            np.array(self.curve_segments[i - 1].robot_lengths(robot_width)) if i > 0 else np.zeros((1, 2))[0]
            for i in range(self.num_of_segments)
        ]
        parts_lengths = [
            np.array([
                seg_lengths[i] + seg.robot_lengths(robot_width, 0, t[0][k])
                for k in range(Segment.NUM_OF_SAMPLES)
            ])
            for (i, seg) in enumerate(self.curve_segments)
        ]

        self.curve_robotdist_left = np.concatenate([dists[:, 0] for dists in parts_lengths])
        self.curve_robotdist_right = np.concatenate([dists[:, 1] for dists in parts_lengths])

        return self.curve_pos

    def write_to_file(self, filename: str = None):
        """
        Writes the curve points to a CSV file.
        :param filename: The wanted filename for the file. Default - curveinfo.csv
        """
        filename = self.info[0] + '.csv' if filename is None else filename
        with open(filename, 'w', newline='') as csvfile:
            fields = ['time', 'x', 'y', 'dx', 'dy', 'heading', 'leftdist', 'rightdist']
            writer = DictWriter(csvfile, fieldnames=fields)
            writer.writeheader()

            for i in range(self.num_of_segments):
                seg = self.curve_segments[i]
                t0, t1 = seg.times
                t = Utils.linspace(t0, t1, samples=Segment.NUM_OF_SAMPLES)[0]
                rangestart = (0 if i == 0 else 1)
                for k in range(rangestart, Segment.NUM_OF_SAMPLES):
                    writer.writerow({
                        'time': t[k],
                        'x': self.curve_pos[100 * i + k, 0],
                        'y': self.curve_pos[100 * i + k, 1],
                        'dx': self.curve_vel[100 * i + k, 0],
                        'dy': self.curve_vel[100 * i + k, 1],
                        'heading': 90 - self.curve_heading[100 * i + k],
                        'leftdist': self.curve_robotdist_left[100 * i + k],
                        'rightdist': self.curve_robotdist_left[100 * i + k]
                    })

