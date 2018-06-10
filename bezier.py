import numpy as np

# Bezier basis matrix
B = np.array([
    [-1, 3, -3, 1],
    [3, -6, 3, 0],
    [-3, 3, 0, 0],
    [1, 0, 0, 0]
])


# Number of samples for every picewise Bézier curve
NUM_OF_SAMPLES = 100


def polynomial(t, xs, ys):
    """
    Returns the Bézier polynomial result for t through the points specified using xs and ys.
    """
    vect = np.array([t ** 3, t ** 2, t, 1]).T
    Bt = np.dot(B, vect)
    return np.array([
        np.dot(xs, Bt),
        np.dot(ys, Bt)
    ])



def flip_curve(bezier, base_width):
    """
    Flip a given Bézier curve.
    :param bezier: The curve to flip
    :return: The flipped curve coordinates
    """
    M = np.array([
        [-1, 0],
        [0, 1]
    ])
    C = np.array([
        base_width * np.ones(NUM_OF_SAMPLES),
        np.zeros(NUM_OF_SAMPLES)
    ])
    return [np.dot(M, a) + C for a in bezier]


def bezier(pts, dts):
    """
    Returns a piecewise Bézier curve, using derivative information for each knot.
    :param pts: Knot array. array<vec2>
    :param dts: Array of derivative information for each knot, assuming
                that dts[i] has the derivative information for pts[i],
                and that dts[i] is "to the left" of pts[i] -- which means
                that if dts[i] = (dx, dy) and pts[i] = (x, y) then dx =< x. array<vec2>
    :return: The curve points run using np.linspace(0, 1, num=1000). array<vec2>
    """
    ld = len(dts)
    lp = len(pts)

    # 1. Have the correct derivative information
    derivatives = []
    for (i, dt) in enumerate(dts):
        if i == 0 or i == ld - 1:
            derivatives.append(dt)
        else:
            derivatives.append(dt)
            derivatives.append(2 * pts[i] - dt)

    # 2. Make an array of 4-point arrays: [p[i], d[i], d[i + 1], p[i + 1]]
    control = np.array([[pts[k], derivatives[2 * k], derivatives[2 * k + 1], pts[k + 1]] for k in range(lp - 1)])

    # 3. Get the x and y coordinates
    xcoords = control[:, :, 0]
    ycoords = control[:, :, 1]

    # 4. Run the function!
    sp = np.linspace(0, 1, num=NUM_OF_SAMPLES)
    m = np.linspace(0, 0, num=0)
    X, _ = np.meshgrid(sp, m, sparse=True)
    res = polynomial(X, xcoords, ycoords).T
    return [[res[c, 0][0].T, res[c, 1][0].T] for c in range(len(res))]

