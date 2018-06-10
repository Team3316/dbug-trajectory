import matplotlib.pyplot as plot
import numpy as np
from bezier import bezier, flip_curve

FEET_IN_METER = 0.3048
fig = plot.figure(figsize=(15, 14.96), dpi=150)
axes = plot.axes()


def plota(points, s):
    for p in points:
        axes.plot(p[0], p[1], s)


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


if __name__ == '__main__':
    setup_plot(8.23, 8.21)
    setup_margins(8.21)

    pts = [
        [0, 0],
        [1, 2],
        [3, 2],
        [4, 4]
    ]

    dts = [
        [0, 1.5],
        [0.5, 2],
        [2.5, 2],
        [4, 2.5]
    ]
    plota(pts, 'rx')

    v4 = bezier(np.array(pts), np.array(dts))
    v4 = flip_curve(v4, 8.21 - 0.91)
    for c in range(len(v4)):
        axes.plot(v4[c][0], v4[c][1], 'green')

    fig.savefig('graph.png')
