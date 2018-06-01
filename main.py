import matplotlib.pyplot as plot
import numpy as np
from scipy.special import binom
from hermite import hermite

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

    pts = np.array([
        [0, 0],
        [0, 3],
        [0.2, 3.4],
        [0.5, 3.5],
        [3, 3.5],
        [3.3, 3.6],
        [3.5, 4],
        [3.5, 7]
    ])

    x = np.linspace(0, 10, num=1000)
    y = np.linspace(0, 10, num=1)
    X, _ = np.meshgrid(x, y, sparse=True)

    v1 = curve(X, pts)
    axes.plot(v1[:, 0], v1[:, 1], 'blue')

    v2 = hermite()
    axes.plot(v2[:, 0], v2[:, 1], 'red')

    fig.savefig('graph.png')
