from trajectory import Trajectory

if __name__ == '__main__':
    traj = Trajectory.from_json('../path1.json', '../mars.json')
    curve = traj.curve()
    print(','.join(['({}, {})'.format(round(p[0], 4), round(p[1], 4)) for p in curve]))
