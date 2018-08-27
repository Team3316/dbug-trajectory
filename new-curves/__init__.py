from trajectory import Trajectory
from curve import CurveType
from math import sqrt

if __name__ == '__main__':
    traj = Trajectory.from_json('../path1.json', '../mars.json')
    position = traj.curve(CurveType.POSITION)
    velocity = [[i / 101, sqrt(v[0] ** 2 + v[1] ** 2)] for (i, v) in enumerate(traj.curve(CurveType.VELOCITY))]

    print(','.join(['({}, {})'.format(round(p[0], 4), round(p[1], 4)) for p in position]))
    print(','.join(['({}, {})'.format(round(p[0], 4), round(p[1], 4)) for p in velocity]))
