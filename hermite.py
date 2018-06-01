import numpy as np

H = np.array([
    [2, -2, 1, 1],
    [-3, 3, -2, -1],
    [0, 0, 1, 0],
    [1, 0, 0, 0]
])

x = np.array([
    0, # x0
    1, # x1
    0, # d0
    3 # d1
]).T

y = np.array([
    0, # y0
    2, # y1
    3, # d0
    0 # d1
]).T

ax = H @ x
ay = H @ y

def p(t):
    v = np.array([t ** 3, t ** 2, t, 1])
    return np.array([
        np.dot(v, ax)[0], # index is needed for 1d vectors
        np.dot(v, ay)[0]
    ])

def hermite ():
    sp = np.linspace(0, 1, num=100)
    r = np.linspace(0, 0, num=0)
    Xs, _ = np.meshgrid(sp, r, sparse=True)
    return p(Xs).T
