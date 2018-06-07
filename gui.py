import numpy as np
from configparser import ConfigParser
from functools import partial
from typing import Tuple
from kivy.app import App
from kivy.config import Config
from kivy.properties import BooleanProperty, ListProperty
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.graphics import Line, Color
from kivy.core.image import Image

'''
TODO:
* Add field image picking
* Create a UI for changing the knot values
'''

Config.set('graphics', 'resizable', False)
Config.write()

# Field Dimensions
field_img_size = Image.load('field.jpg').size  # Image field dimensions, in pixels.
field_dimensions = (8.23, 16.46)  # Real-world field dimensions, in meters.


class Utils(object):
    lb_corner: Tuple[int, int] = None
    rt_corner: Tuple[int, int] = None

    @classmethod
    def screen_to_field(cls, p: Tuple[int, int]):
        wi, hi = field_img_size
        wr, hr = field_dimensions
        x = (p[0] - cls.lb_corner[0]) * (wr / wi)
        y = (p[1] - cls.lb_corner[1]) * (hr / hi)
        return x, y

    @classmethod
    def world_to_field(cls, p: Tuple[int, int]):
        wi, hi = field_img_size
        wr, hr = field_dimensions
        x = (wi / wr) * p[0] + cls.lb_corner[0]
        y = (hi / hr) * p[1] + cls.lb_corner[1]
        return x, y

    @classmethod
    def color(cls, r: float, g: float, b: float, a: float = 1.0):
        return Color(r / 255.0, g / 255.0, b / 255.0, a)

    @classmethod
    def draw_point(cls, widget, color: Tuple[float, float, float], point: Tuple[int, int]):
        with widget.canvas:
            Utils.color(r=color[0], g=color[1], b=color[2])
            Line(cirlce=(point[0], point[1], 3), width=3)


class LinePlayground(FloatLayout):
    pts = ListProperty([])
    dts = ListProperty([])
    field_set = ListProperty([False, False])

    def __init__(self):
        super(LinePlayground, self).__init__()
        self.size = field_img_size

    def on_touch_down(self, touch):
        if super(LinePlayground, self).on_touch_down(touch):
            return True
        if self.field_set[0] and self.field_set[1]:
            touch.grab(self)
            self.pts.append(touch.pos)
        return True

    def on_touch_move(self, touch):
        if touch.grab_current is self:
            self.pts[-1] = touch.pos
            return True
        return super(LinePlayground, self).on_touch_move(touch)

    def on_touch_up(self, touch):
        if Utils.lb_corner is None:  # Handle left-bottom field corner
            Utils.lb_corner = touch.pos
            self.field_set[0] = True
            with self.canvas:
                Utils.color(r=0, g=255, b=0)
                Line(circle=(touch.pos[0], touch.pos[1], 3), width=3)
        elif Utils.rt_corner is None:  # Handle right-top field corner
            Utils.rt_corner = touch.pos
            self.field_set[1] = True
            with self.canvas:
                Utils.color(r=0, g=255, b=0)
                Line(circle=(touch.pos[0], touch.pos[1], 3), width=3)
        if touch.grab_current is self and self.field_set[0] and self.field_set[1]:
            touch.ungrab(self)
            with self.canvas:
                Utils.color(r=244.0, g=67.0, b=54.0)
                Line(circle=(touch.pos[0], touch.pos[1], 3), width=3)
            return True
        return super(LinePlayground, self).on_touch_up(touch)


class PlannerApp(App):
    def __init__(self):
        super(PlannerApp, self).__init__()
        self.config = ConfigParser()

    def save_config(self, *largs):
        self.config['field'] = {
            'lb_x': Utils.lb_corner[0],
            'lb_y': Utils.lb_corner[1],
            'rt_x': Utils.rt_corner[0],
            'rt_y': Utils.rt_corner[1],
            'world_width': field_dimensions[0],
            'world_height': field_dimensions[1]
        }
        with open('planner.ini', 'w') as cfile:
            self.config.write(cfile)

    def import_config(self, *largs):
        global field_dimensions
        print(self.config.read('planner.ini'))
        field = self.config.read('planner.ini')['field']
        field_dimensions = (float(field['world_width']), float(field['world_height']))
        Utils.lb_corner = (int(field['lb_x']), int(field['lb_y']))
        Utils.rt_corner = (int(field['rt_x']), int(field['rt_y']))

    def test_data(self, field, *largs):
        pt = Utils.world_to_field((1, 0))
        with field.canvas:
            Utils.color(r=0, g=255, b=0)
            Line(circle=(pt[0], pt[1], 3), width=3)

    def build(self):
        layout = BoxLayout(size_hint=(1,  None), height=50)

        cexport = Button(text='Export Field Settings')
        cexport.bind(on_release=self.save_config)
        layout.add_widget(cexport)

        cimport = Button(text='Import Field Settings')
        cimport.bind(on_release=self.import_config)
        layout.add_widget(cimport)

        field = LinePlayground()

        test = Button(text='Draw test data')
        test.bind(on_release=partial(self.test_data, field))
        layout.add_widget(test)

        root = BoxLayout(orientation='vertical')
        root.add_widget(field)
        root.add_widget(layout)
        return root


if __name__ == '__main__':
    PlannerApp().run()