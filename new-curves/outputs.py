import matplotlib.pyplot as plot

from matplotlib.patches import Rectangle
from abc import ABC, abstractmethod
from .trajectory import Trajectory
from numpy import arange
from .curve import Curve


class Output(ABC):
    def __init__(self, trajectory: Trajectory):
        self.trajectory = trajectory

    @abstractmethod
    def render(self):
        pass


class PlotOutput(Output, ABC):
    FEET_IN_METER = 0.3048

    def __init__(self, trajectory: Trajectory, field_width: float, field_height: float):
        super(PlotOutput, self).__init__(trajectory)

        self.width = field_width
        self.height = field_height

        self.fig = plot.figure(figsize=(15, 14.96), dpi=300)
        self.axes = plot.axes()

    def setup_plot(self):
        # x tick
        plot.xlim(0, self.height)
        plot.xticks(fontsize=13, rotation=90)
        self.axes.set_xticks(arange(0, self.height, PlotOutput.FEET_IN_METER))

        # y tick
        plot.ylim(0, self.width + 3 * PlotOutput.FEET_IN_METER)
        plot.yticks(fontsize=13)
        self.axes.set_yticks(arange(0, self.width + 3 * PlotOutput.FEET_IN_METER, PlotOutput.FEET_IN_METER))

        # gridlines
        self.axes.grid(which='both')

        # Bottom margin
        bx = [0.75, 0]
        by = [0, 0.91]
        self.axes.plot(by[0:2], bx[0:2], 'k-')

        # Top margin
        tx = [0.75, 0]
        ty = [self.height, self.height - 0.91]
        self.axes.plot(ty[0:2], tx[0:2], 'k-')

    def setup_obstacles(self):
        ft_m = PlotOutput.FEET_IN_METER

        # Switch
        switch = Rectangle((2.165, 3.556), 3.89, 1.4224)
        self.axes.add_patch(switch)

        # Scale
        platform = Rectangle((2.41935, 6.6413), 3.2893, 4.5 * ft_m, color='r')
        self.axes.add_patch(platform)
        scale_left = Rectangle((1.8179, 7.61), 3 * ft_m, 4 * ft_m, color='#00FF00')
        self.axes.add_patch(scale_left)
        scale_right = Rectangle((1.8179 + 12 * ft_m, 7.61), 3 * ft_m, 4 * ft_m, color='#00FF00')
        self.axes.add_patch(scale_right)

    def render(self):
        curve = self.trajectory.curve()

