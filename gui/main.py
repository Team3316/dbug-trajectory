from configparser import ConfigParser
from kivy.app import App
from kivy.config import Config
from kivy.graphics import Line
from gui.utils import Utils
from gui.point_widget import Point

'''
TODO:
* Add field image picking
* Create a UI for changing the knot values
'''

Config.set('graphics', 'resizable', False)
Config.write()


class PlannerApp(App):
    def __init__(self, **kwargs):
        super(PlannerApp, self).__init__(**kwargs)

        # Field Dimensions
        field_dimensions = (8.23, 16.46)  # Real-world field dimensions, in meters.

        self.utils = Utils(field_dimensions)

    def save_config(self, *largs):
        self.utils.to_config()

    def import_config(self, *largs):
        self.utils = Utils.from_config()

    def test_data(self, *largs):
        field = self.root.ids["'field'"]
        point = Point(8.23, 0)
        field.add_widget(point)
        # with field.canvas:
        #     Utils.color(r=0, g=255, b=0)
        #     Line(circle=(pt[0], pt[1], 3), width=3)


if __name__ == '__main__':
    PlannerApp().run()
