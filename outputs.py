import matplotlib.pyplot as plot

from numpy import arange, ndarray, array as nparray
from trajectory import Trajectory, RobotSide
from matplotlib.patches import Rectangle
from math import sin, cos, radians, sqrt
from abc import ABC, abstractmethod
from curve import CurveType
from csv import DictWriter
from typing import List


class Output(ABC):
    def __init__(self, trajectory: Trajectory):
        self.trajectory = trajectory

    @abstractmethod
    def render(self):
        pass


class PlotOutput(Output, ABC):
    FEET_IN_METER = 0.3048

    def __init__(self, trajectory: Trajectory, field_width: float, field_height: float):
        super(PlotOutput, self).__init__(trajectory)

        self.width = field_width
        self.height = field_height

        self.fig = plot.figure(figsize=(15, 14.96), dpi=300)
        self.axes = plot.axes()

    def setup_plot(self):
        # x tick
        plot.xlim(0, self.height)
        plot.xticks(fontsize=13, rotation=90)
        self.axes.set_xticks(arange(0, self.height, PlotOutput.FEET_IN_METER))

        # y tick
        plot.ylim(0, self.width + 3 * PlotOutput.FEET_IN_METER)
        plot.yticks(fontsize=13)
        self.axes.set_yticks(arange(0, self.width + 3 * PlotOutput.FEET_IN_METER, PlotOutput.FEET_IN_METER))

        # gridlines
        self.axes.grid(which='both')

        # Bottom margin
        bx = [0.75, 0]
        by = [0, 0.91]
        self.axes.plot(by[0:2], bx[0:2], 'k-')

        # Top margin
        tx = [0.75, 0]
        ty = [self.height, self.height - 0.91]
        self.axes.plot(ty[0:2], tx[0:2], 'k-')

    def setup_obstacles(self):
        ft_m = PlotOutput.FEET_IN_METER

        # Switch
        switch = Rectangle((2.165, 3.556), 3.89, 1.4224)
        self.axes.add_patch(switch)

        # Scale
        platform = Rectangle((2.41935, 6.6413), 3.2893, 4.5 * ft_m, color='r')
        self.axes.add_patch(platform)
        scale_left = Rectangle((1.8179, 7.61), 3 * ft_m, 4 * ft_m, color='#00FF00')
        self.axes.add_patch(scale_left)
        scale_right = Rectangle((1.8179 + 12 * ft_m, 7.61), 3 * ft_m, 4 * ft_m, color='#00FF00')
        self.axes.add_patch(scale_right)

    def plot_headings(self, shift_x: float, curve: ndarray, headings: List[float], resolution: int = 10):
        lh = len(headings)
        for i in range(int(lh / resolution)):
            a = headings[resolution * i]
            plot.arrow(
                curve[resolution * i, 0] + shift_x,
                curve[resolution * i, 1],
                0.5 * cos(radians(a)),
                0.5 * sin(radians(a)),
                fc='b',
                ec='b',
                head_width=0.03,
                length_includes_head=True
            )

    def plot_control_points(self, shift_x: float, control_points: ndarray):
        for points in control_points:
            p0, v0, a0, p1, v1, a1 = points

            self.axes.plot(p0[0] + shift_x, p0[1], 'bo')
            self.axes.plot(v0[0] + p0[0] + shift_x, v0[1] + p0[1], 'rx')
            self.axes.plot(a0[0] + p0[0] + shift_x, a0[1] + p0[1], 'g8')

            self.axes.plot(p1[0] + shift_x, p1[1], 'bo')
            self.axes.plot(v1[0] + p1[0] + shift_x, v1[1] + p1[1], 'rx')
            self.axes.plot(a1[0] + p1[0] + shift_x, a1[1] + p1[1], 'g8')

    def render(self):
        self.setup_plot()
        self.setup_obstacles()

        shift_x = 0.91 + self.trajectory.robot.robot_info[3] / 2

        left_curve = self.trajectory.robot_curve(CurveType.POSITION, RobotSide.LEFT)
        middle_curve = self.trajectory.curve(CurveType.POSITION)
        right_curve = self.trajectory.robot_curve(CurveType.POSITION, RobotSide.RIGHT)

        self.axes.plot(left_curve[:, 0] + shift_x, left_curve[:, 1], 'magenta')
        self.axes.plot(middle_curve[:, 0] + shift_x, middle_curve[:, 1], '#00FF00')
        self.axes.plot(right_curve[:, 0] + shift_x, right_curve[:, 1], 'magenta')

        self.plot_headings(shift_x, middle_curve, self.trajectory.headings()[0])
        self.plot_control_points(shift_x, self.trajectory.control_points())

        self.fig.savefig('graph.png')


class DesmosOutput(Output):
    def format(self, curve: ndarray, i: int = 1) -> str:
        points = ['({}, {})'.format(round(v[0], 4), round(v[i], 4)) for v in curve]
        return ','.join(points)

    def render(self):
        print('Position:')
        print(self.format(self.trajectory.curve(CurveType.POSITION)))

        print('Velocity vectors:')
        print(self.format(self.trajectory.curve(CurveType.VELOCITY)))

        print('Velocity:')
        print(self.format(self.trajectory.speed()))

        print('Left position:')
        print(self.format(self.trajectory.robot_curve(CurveType.POSITION, RobotSide.LEFT)))

        print('Right position:')
        print(self.format(self.trajectory.robot_curve(CurveType.POSITION, RobotSide.RIGHT)))

        free_speed = self.trajectory.robot.chassis_info[0]
        vleft = self.trajectory.robot_speeds(RobotSide.LEFT)
        vright = self.trajectory.robot_speeds(RobotSide.RIGHT)

        print('Left velocity:')
        print(self.format(vleft))

        print('Right velocity:')
        print(self.format(vright))

        print('Left output:')
        print(self.format(nparray([
            [v[0], v[1] / free_speed]
            for v in vleft
        ])))

        print('Right output:')
        print(self.format(nparray([
            [v[0], v[1] / free_speed]
            for v in vright
        ])))

        print('Middle distances:')
        print(self.format(self.trajectory.distance()))


class CSVOutput(Output):
    FIELDS = ['time', 'x', 'y', 'dx', 'dy', 'heading', 'vleft', 'vright', 'acceleration']

    def __init__(self, trajectory: Trajectory, filename: str = None):
        super().__init__(trajectory)

        fname = trajectory.name + '.csv' if filename is None else filename
        self.file = open(fname, 'w', newline='')
        self.writer = DictWriter(self.file, fieldnames=CSVOutput.FIELDS)

    def render(self):
        self.writer.writeheader()

        num_of_samples = self.trajectory.num_of_segments * (Trajectory.SAMPLE_SIZE + 1)

        middle_position = self.trajectory.curve(CurveType.POSITION, concat=True)
        middle_velocity = self.trajectory.curve(CurveType.VELOCITY, concat=True)
        middle_acceleration = self.trajectory.curve(CurveType.ACCELERATION, concat=True)
        headings = self.trajectory.headings()[0]
        left_speed = self.trajectory.robot_speeds(RobotSide.LEFT)
        right_speed = self.trajectory.robot_speeds(RobotSide.RIGHT)

        for i in range(num_of_samples):
            self.writer.writerow({
                'time': left_speed[i][0],
                'x': middle_position[i, 0],
                'y': middle_position[i, 1],
                'dx': middle_velocity[i, 0],
                'dy': middle_velocity[i, 1],
                'heading': 90 - headings[i],
                'vleft': left_speed[i][1],
                'vright': right_speed[i][1],
                'acceleration': sqrt(middle_acceleration[i, 0] ** 2 + middle_acceleration[i, 1] ** 2)
            })
