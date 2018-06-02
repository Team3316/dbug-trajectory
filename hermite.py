import numpy as np

# Hermite basis matrix
H = np.array([
    [2, -2, 1, 1],
    [-3, 3, -2, -1],
    [0, 0, 1, 0],
    [1, 0, 0, 0]
])

def vcs (p0, p1, p2, p3):
    x = np.array([p0[0], p2[0], p1[0] - p0[0], p3[0] - p2[0]]).T
    y = np.array([p0[1], p2[1], p1[1] - p1[1], p3[1] - p2[1]]).T
    return (H @ x, H @ y)

def p (t, ax, ay):
    v = np.array([t ** 3, t ** 2, t, 1])
    return np.array([
        np.dot(v, ax)[0], # index is needed for 1d vectors
        np.dot(v, ay)[0]
    ])

def hermite (ax, ay):
    sp = np.linspace(0, 1, num=1000)
    r = np.linspace(0, 0, num=0)
    Xs, _ = np.meshgrid(sp, r, sparse=True)
    return p(Xs, ax, ay).T
