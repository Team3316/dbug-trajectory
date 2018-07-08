import matplotlib.pyplot as plot
from matplotlib.patches import Rectangle
import numpy as np
from bezier import Bezier
from math import sin, cos, radians

FEET_IN_METER = 0.3048
fig = plot.figure(figsize=(15, 14.96), dpi=150)
axes = plot.axes()


def plota(points, origin, s):
    for p in points:
        axes.plot(p[0] + origin[0], p[1] + origin[1], s)


def setup_plot(width, height):
    plot.xlim(0, height)
    plot.xticks(fontsize=13, rotation=90)
    axes.set_xticks(np.arange(0, height, FEET_IN_METER))

    plot.ylim(0, width)
    plot.yticks(fontsize=13)
    axes.set_yticks(np.arange(0, width, FEET_IN_METER))

    axes.grid(which='both')


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
    switch = Rectangle((2.165, 3.556), 3.89, 1.4224)
    axes.add_patch(switch)


def plot_headings(headings, resolution: int = 10):
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

    pts = [
        [0, 0],
        [0.895, 4.572]
    ]

    dts = [
        [0, 1.5],
        [-1, 4.572]
    ]

    times = [
        0,  # First point always is at time 0!!
        1
    ]

    robotwidth = 0.7
    origin = [0.91 + robotwidth / 2, 0]

    bezier = Bezier(pts=pts, dts=dts, times=times)

    bezier.origin = origin
    plota(pts, origin, 'rx')
    plota(dts, origin, 'bo')

    bezier.gen_constraints()
    bezier.gen_segments()
    curve = bezier.curve(0.7, flip=True, basewidth=8.21)

    axes.plot(curve[:, 0], curve[:, 1], '#00ff00')
    plot_headings(bezier.curve_heading)

    axes.plot(bezier.curve_robotl[:, 0], bezier.curve_robotl[:, 1], 'magenta')
    axes.plot(bezier.curve_robotr[:, 0], bezier.curve_robotr[:, 1], 'magenta')

    fig.savefig('graph.png')
    bezier.write_to_file()
