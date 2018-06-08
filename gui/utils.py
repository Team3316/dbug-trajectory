from typing import Tuple
from kivy.graphics import Line, Color

class Utils(object):

    def __init__(self, field_dimensions: Tuple[float, float]):
        self.lb_corner: Tuple[int, int] = None
        self.rt_corner: Tuple[int, int] = None
        self.field_dimensions = field_dimensions

    @property
    def field_img_size(self):
        if self.rt_corner is None or self.lb_corner is None:
            return 0, 0
        return self.rt_corner[0] - self.lb_corner[0], self.rt_corner[1] - self.lb_corner[1]

    def screen_to_field(self, p: Tuple[int, int]):
        wi, hi = self.field_img_size
        wr, hr = self.field_dimensions
        x = (p[0] - self.lb_corner[0]) * (wr / wi)
        y = (p[1] - self.lb_corner[1]) * (hr / hi)
        return x, y

    def world_to_field(self, p: Tuple[int, int]):
        wi, hi = self.field_img_size
        wr, hr = self.field_dimensions
        x = (wi / wr) * p[0] + self.lb_corner[0]
        y = (hi / hr) * p[1] + self.lb_corner[1]
        return x, y

    @classmethod
    def color(cls, r: float, g: float, b: float, a: float = 1.0):
        return Color(r / 255.0, g / 255.0, b / 255.0, a)

    def draw_point(self, widget, color: Tuple[float, float, float], point: Tuple[int, int]):
        with widget.canvas:
            Utils.color(r=color[0], g=color[1], b=color[2])
            Line(cirlce=(point[0], point[1], 3), width=3)
