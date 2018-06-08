from kivy.properties import ListProperty
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import Line
from kivy.app import App
from gui.utils import Utils

class LinePlayground(FloatLayout):
    pts = ListProperty([])
    dts = ListProperty([])

    def __init__(self, **kwargs):
        super(LinePlayground, self).__init__(**kwargs)
        self.utils = App.get_running_app().utils
        self.size = self.utils.field_img_size

    def on_touch_down(self, touch):
        if super(LinePlayground, self).on_touch_down(touch):
            return True
        if self.utils.lb_corner is not None and self.utils.rt_corner is not None:
            touch.grab(self)
            self.pts.append(touch.pos)
        return True

    def on_touch_move(self, touch):
        if touch.grab_current is self:
            self.pts[-1] = touch.pos
            return True
        return super(LinePlayground, self).on_touch_move(touch)

    def on_touch_up(self, touch):
        if self.utils.lb_corner is None:  # Handle left-bottom field corner
            self.utils.lb_corner = touch.pos
            with self.canvas:
                Utils.color(r=0, g=255, b=0)
                Line(circle=(touch.pos[0], touch.pos[1], 3), width=3)
        elif self.utils.rt_corner is None:  # Handle right-top field corner
            self.utils.rt_corner = touch.pos
            with self.canvas:
                Utils.color(r=0, g=255, b=0)
                Line(circle=(touch.pos[0], touch.pos[1], 3), width=3)
        elif touch.grab_current is self:
            touch.ungrab(self)
            with self.canvas:
                Utils.color(r=244.0, g=67.0, b=54.0)
                Line(circle=(touch.pos[0], touch.pos[1], 3), width=3)
            return True
        return super(LinePlayground, self).on_touch_up(touch)
