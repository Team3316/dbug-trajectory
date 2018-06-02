import matplotlib.pyplot as plot
import numpy as np
from bezier import bezier

fig = plot.figure()
axes = plot.axes()

def plota(points, s):
    for p in points:
        axes.plot(p[0], p[1], s)

if __name__ == '__main__':
    plot.xlim(0, 10)
    plot.ylim(0, 10)

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
    plota(dts, 'bx')

    v4 = bezier(np.array(pts), np.array(dts))
    for c in range(len(v4[:, 0])):
        axes.plot(v4[c, 0][0], v4[c, 1][0], 'green')

    fig.savefig('graph.png')
