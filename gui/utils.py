from configparser import ConfigParser
from typing import Tuple
from kivy.graphics import Line, Color


class Utils(object):
    def __init__(self, field_dimensions: Tuple[float, float]):
        self.lb_corner: Tuple[int, int] = None
        self.rt_corner: Tuple[int, int] = None
        self.field_dimensions = field_dimensions

    @classmethod
    def from_config(cls, filename: str = 'planner.ini'):
        config = ConfigParser()
        config.read(filename)
        field = config['field']
        u = cls((float(field['world_width']), float(field['world_height'])))
        u.lb_corner = (int(field['lb_x']), int(field['lb_y']))
        u.rt_corner = (int(field['rt_x']), int(field['rt_y']))
        return u

    def to_config(self, filename: str = 'planner.ini'):
        config = ConfigParser()
        config['field'] = {
            'lb_x': self.lb_corner[0],
            'lb_y': self.lb_corner[1],
            'rt_x': self.rt_corner[0],
            'rt_y': self.rt_corner[1],
            'world_width': self.field_dimensions[0],
            'world_height': self.field_dimensions[1]
        }
        with open(filename, 'w') as cfile:
            config.write(cfile)

    @property
    def field_img_size(self):
        if self.rt_corner is None or self.lb_corner is None:
            return 0, 0
        return self.rt_corner[0] - self.lb_corner[0], self.rt_corner[1] - self.lb_corner[1]

    def screen_to_world(self, p: Tuple[int, int]):
        wi, hi = self.field_img_size
        wr, hr = self.field_dimensions
        x = (p[0] - self.lb_corner[0]) * (wr / wi)
        y = (p[1] - self.lb_corner[1]) * (hr / hi)
        return x, y

    def world_to_screen(self, p: Tuple[int, int]):
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
