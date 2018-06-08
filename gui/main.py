from configparser import ConfigParser
from kivy.app import App
from kivy.config import Config
from kivy.graphics import Line
from kivy.core.image import Image
from gui.utils import Utils

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
        # field_img_size = Image.load('field.jpg').size  # Image field dimensions, in pixels.
        field_dimensions = (8.23, 16.46)  # Real-world field dimensions, in meters.

        self.utils = Utils(field_dimensions)

    def save_config(self, *largs):
        config = ConfigParser()
        config['field'] = {
            'lb_x': self.utils.lb_corner[0],
            'lb_y': self.utils.lb_corner[1],
            'rt_x': self.utils.rt_corner[0],
            'rt_y': self.utils.rt_corner[1],
            'world_width': self.utils.field_dimensions[0],
            'world_height': self.utils.field_dimensions[1]
        }
        with open('planner.ini', 'w') as cfile:
            config.write(cfile)

    def import_config(self, *largs):
        config = ConfigParser()
        config.read('planner.ini')
        field = config['field']
        self.utils.field_dimensions = (float(field['world_width']), float(field['world_height']))
        self.utils.lb_corner = (int(field['lb_x']), int(field['lb_y']))
        self.utils.rt_corner = (int(field['rt_x']), int(field['rt_y']))

    def test_data(self, *largs):
        pt = self.utils.world_to_field((8.23, 0))
        print(pt)
        field = self.root.ids["'field'"]
        with field.canvas:
            Utils.color(r=0, g=255, b=0)
            Line(circle=(pt[0], pt[1], 3), width=3)


if __name__ == '__main__':
    PlannerApp().run()
