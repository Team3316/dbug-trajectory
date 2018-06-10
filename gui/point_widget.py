from enum import Enum
from kivy.app import App
from kivy.uix.anchorlayout import AnchorLayout
from kivy.graphics import Line
import numpy as np


class ReferenceFrame(Enum):
    SCREEN = 1
    WORLD = 2


class Point(AnchorLayout):
    def __init__(self, x: float, y: float, frame: ReferenceFrame = ReferenceFrame.WORLD):
        self.tuple = (x, y)

        self.frame = frame
        self.size = (5, 5)
        self.pos = self.transform(to=ReferenceFrame.SCREEN)

    def transform(self, to: ReferenceFrame):
        utils = App.get_running_app().utils
        if to == ReferenceFrame.WORLD:
            return utils.screen_to_world(self.tuple)
        elif to == ReferenceFrame.SCREEN:
            return utils.world_to_screen(self.tuple)
