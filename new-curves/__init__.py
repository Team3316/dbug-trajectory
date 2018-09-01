from trajectory import Trajectory, RobotSide
from numpy import concatenate as npconcat
from curve import CurveType
from math import sqrt, cos, sin, radians

if __name__ == '__main__':
    traj = Trajectory.from_json('../path1.json', '../mars.json')
    pos_curve = traj.curve(CurveType.POSITION)
    vel_curve = traj.curve(CurveType.VELOCITY)

    segments = traj.curve(CurveType.VELOCITY, concat=False)
    speed = npconcat([
        [
            [
                i + (j / 100),
                sqrt(v[0] ** 2 + v[1] ** 2) - sqrt(seg[0][0] ** 2 + seg[0][1] ** 2)
            ]
            for (j, v) in enumerate(seg)
        ]
        for (i, seg) in enumerate(segments)
    ])

    headings = traj.headings()
    left_pos = [
        p + [
            -sin(radians(headings[i])),
            cos(radians(headings[i]))
        ]
        for (i, p) in enumerate(pos_curve)
    ]
    right_pos = [
        p - [
            -sin(radians(headings[i])),
            cos(radians(headings[i]))
        ]
        for (i, p) in enumerate(pos_curve)
    ]

    desmosify = lambda c: print(','.join(['({}, {})'.format(round(p[0], 4), round(p[1], 4)) for p in c]))

    print('Position:')
    desmosify(pos_curve)

    print('Velocity:')
    desmosify(vel_curve)

    print('Speed (magnitude of velocity):')
    desmosify(speed)

    print('Left position:')
    desmosify(traj.robot_curve(CurveType.POSITION, RobotSide.LEFT))

    print('Right position:')
    desmosify(traj.robot_curve(CurveType.POSITION, RobotSide.RIGHT))
