from trajectory import Trajectory, RobotSide
from curve import CurveType

if __name__ == '__main__':
    traj = Trajectory.from_json('path1.json', 'mars.json')
    free_speed = traj.robot.chassis_info[0]

    pos_curve = traj.curve(CurveType.POSITION)

    desmosify = lambda c: print(','.join(['({}, {})'.format(round(p[0], 4), round(p[1], 4)) for p in c]))

    print('Position:')
    desmosify(pos_curve)


    print('Left position:')
    desmosify(traj.robot_curve(CurveType.POSITION, RobotSide.LEFT))

    print('Right position:')
    desmosify(traj.robot_curve(CurveType.POSITION, RobotSide.RIGHT))

    print('Left speed:')
    desmosify(traj.robot_speeds(RobotSide.LEFT))

    print('Right speed:')
    desmosify(traj.robot_speeds(RobotSide.RIGHT))
