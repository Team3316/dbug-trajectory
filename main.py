import matplotlib.pyplot as plot
from matplotlib.patches import Rectangle
import numpy as np
from bezier import Bezier
from robot import Robot
from segment import Segment
from utils import Utils
from math import sin, cos, radians

FEET_IN_METER = 0.3048
fig = plot.figure(figsize=(15, 14.96), dpi=150)
axes = plot.axes()

fig2 = plot.figure(figsize=(15, 14.96), dpi=150)
axes2 = plot.axes()

def plota(points, o, s):
    for p in points:
        axes.plot(p[0] + o[0], p[1] + o[1], s)


def setup_plot(width, height):
    plot.xlim(0, height)
    plot.xticks(fontsize=13, rotation=90)
    axes.set_xticks(np.arange(0, height, FEET_IN_METER))

    plot.ylim(0, width + 3 * FEET_IN_METER)
    plot.yticks(fontsize=13)
    axes.set_yticks(np.arange(0, width + 3 * FEET_IN_METER, FEET_IN_METER))

    axes.grid(which='both')

    plot.xlim(0, 1)
    plot.xticks(fontsize=13, rotation=90)
    axes2.set_xticks(np.arange(0, 1, 0.01))

    plot.ylim(0, width)
    plot.yticks(fontsize=13)
    axes2.set_yticks(np.arange(0, width, 0.1))

    axes2.grid(which='both')


def setup_margins(height):
    # Bottom margin
    bx = [0.75, 0]
    by = [0, 0.91]
    axes.plot(by[0:2], bx[0:2], 'k-')

    # Top margin
    tx = [0.75, 0]
    ty = [height, height - 0.91]
    axes.plot(ty[0:2], tx[0:2], 'k-')


def setup_obstacles():
    # Switch
    switch = Rectangle((2.165, 3.556), 3.89, 1.4224)
    axes.add_patch(switch)

    # Scale
    platform = Rectangle((2.41935, 6.6413), 3.2893, 4.5 * FEET_IN_METER, color='r')
    axes.add_patch(platform)
    scale_left = Rectangle((1.8179, 7.61), 3 * FEET_IN_METER, 4 * FEET_IN_METER, color='#00FF00')
    axes.add_patch(scale_left)
    scale_right = Rectangle((1.8179 + 12 * FEET_IN_METER, 7.61), 3 * FEET_IN_METER, 4 * FEET_IN_METER, color='#00FF00')
    axes.add_patch(scale_right)


def plot_headings(curve, headings, resolution: int = 10):
    lh = len(headings)
    for i in range(int(lh / resolution)):
        a = headings[resolution * i]
        plot.arrow(
            curve[resolution * i, 0],
            curve[resolution * i, 1],
            0.5 * cos(radians(a)),
            0.5 * sin(radians(a)),
            fc='b',
            ec='b',
            head_width=0.03,
            length_includes_head=True
        )


if __name__ == '__main__':
    setup_plot(8.23, 8.21)
    setup_margins(8.21)
    setup_obstacles()

    robotwidth = 0.7
    origin = [0.91 + robotwidth / 2, 0]

    mars = Robot.from_json("mars.json")
    mars.load_path("path1.json")

    scale_rr = Bezier.from_json('path1.json')
    scale_rr.gen_constraints()
    scale_rr.gen_segments()

    plota(scale_rr.pts, origin, 'ro')
    plota(scale_rr.complete_derivatives, origin, 'bo')

    curve2 = scale_rr.curve(0.7)

    axes.plot(curve2[:, 0], curve2[:, 1], '#00ff00')
    plot_headings(curve2, scale_rr.curve_heading)

    axes.plot(scale_rr.curve_robotl[:, 0], scale_rr.curve_robotl[:, 1], 'magenta')
    axes.plot(scale_rr.curve_robotr[:, 0], scale_rr.curve_robotr[:, 1], 'magenta')

    axes2.plot(Utils.linspace(0, 1, Segment.NUM_OF_SAMPLES * scale_rr.num_of_segments).T, scale_rr.curve_robotvleft, 'magenta')
    axes2.plot(Utils.linspace(0, 1, Segment.NUM_OF_SAMPLES * scale_rr.num_of_segments).T, scale_rr.curve_robotvright, 'blue')

    fig.savefig('graph.png')
    fig2.savefig('velocities.png')
    scale_rr.write_to_file()
