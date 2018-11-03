from outputs import PlotOutput, DesmosOutput, CSVOutput, SimpulationOutput
from trajectory import Trajectory

if __name__ == '__main__':
    traj = Trajectory.from_json('path1.json', 'mars.json')
    print(traj.robot.max_acceleration())

    # plot = PlotOutput(trajectory=traj, field_width=8.23, field_height=8.21)
    # plot.render()
    #
    # desmos = DesmosOutput(trajectory=traj)
    # desmos.render()
    #
    # csv = CSVOutput(trajectory=traj)
    # csv.render()

    sim = SimpulationOutput(trajectory=traj, field_width=8.23, field_height=8.21)
    sim.render()
