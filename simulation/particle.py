import pygame
import pygame.gfxdraw

from math import cos, sin
from typing import Tuple

class Particle:
    def __init__(
            self,
            point: Tuple[float, float],
            radius: float,
            origin: Tuple[float, float],
            color: Tuple[int, int, int] = (255, 0, 0)
    ):
        self.x = point[0] + origin[0]
        self.y = point[1] + origin[1]
        self.radius = radius
        self.color = color
        self.thickness = 1
        self.speed = 0
        self.theta = 0

    def display(self, screen):
        # pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius, self.thickness)
        pygame.gfxdraw.pixel(screen, int(self.x), int(self.y), self.color)

    def move(self):
        self.x += cos(self.theta) * self.speed
        self.y -= sin(self.theta) * self.speed