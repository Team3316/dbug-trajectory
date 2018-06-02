import matplotlib.pyplot as plot
import numpy as np
from scipy.special import binom
from hermite import hermite, vcs
from bezier import bezier

"""
TODO:
* Make this an actual program
* Move waypoint normalization into their own function
"""
fig = plot.figure()
axes = plot.axes()


def plota(points, s):
    for p in points:
        axes.plot(p[0], p[1], s)

# Bernstein Polynomials
def B(i, d):
    return lambda t: binom(d, i) * (t ** i) * ((1 - t) ** (d - i))


# Bezier curve for n points
def curve(t, ps):
    l = len(ps)
    polys = [ps[i] * np.transpose(B(i, l - 1)(t)) for i in range(l)]
    return np.array(np.sum(polys, axis=0))

if __name__ == '__main__':
    plot.xlim(0, 10)
    plot.ylim(0, 10)

    # ps = [
    #     [0, 0],
    #     [0, 3],
    #     [0.2, 3.4],
    #     [0.5, 3.5],
    #     [3, 3.5],
    #     [3.3, 3.6],
    #     [3.5, 4],
    #     [3.5, 7]
    # ]
    # plota(ps, 'gx')
    # pts = np.array(ps)
    #
    # x = np.linspace(0, 10, num=1000)
    # y = np.linspace(0, 10, num=1)
    # X, _ = np.meshgrid(x, y, sparse=True)
    #
    # v1 = curve(X, pts)
    # axes.plot(v1[:, 0], v1[:, 1], 'blue')
    #
    # ax, ay = vcs(ps[0], ps[1], ps[2], ps[3])
    # v2 = hermite(ax, ay)
    # axes.plot(v2[:, 0], v2[:, 1], 'red')
    #
    # ax, ay = vcs(ps[2], ps[3], ps[4], ps[5])
    # v3 = hermite(ax, ay)
    # axes.plot(v3[:, 0], v3[:, 1], 'red')

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
