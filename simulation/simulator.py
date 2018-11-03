import pygame
import pygame.gfxdraw

from .particle import Particle
from typing import Tuple, List
from math import pi

class DBugSimulator:
    def __init__(self, win_dimensions: Tuple[int, int], name: str):
        self.name = name
        self.screen = pygame.display.set_mode(win_dimensions)
        self.running = False
        self.p = Particle((0, 0), 1, origin=(50, win_dimensions[1] - 50))
        self.dims = win_dimensions

        self.curves = []
        self.timer = 0

    def configure(self):
        pygame.init()
        pygame.display.set_caption('Trajectory: %s' % self.name)

    def initial_frame(self):
        self.p.speed = 0.2
        self.p.theta = pi / 6
        self.p.display(screen=self.screen)

        pygame.draw.line(self.screen, (0, 0, 0), (50, self.dims[1] - 50), (self.dims[0] - 50, self.dims[1] - 50))

    def render_frame(self, offset: int):
        border = offset / 2
        pygame.draw.line(self.screen, (0, 0, 0), (border, border), (border, self.dims[1] - border))
        pygame.draw.line(self.screen, (0, 0, 0), (border, self.dims[1] - border), (self.dims[0] - border, self.dims[1] - border))
        pygame.draw.line(self.screen, (0, 0, 0), (self.dims[0] - border, self.dims[1] - border), (self.dims[0] - border, border))
        pygame.draw.line(self.screen, (0, 0, 0), (self.dims[0] - border, border), (border, border))

    def render_points(self, points: List[Particle]):
        self.curves.append(points)

    def render(self):
        self.timer += 1

        for curve in self.curves:
            lc = len(curve)
            for i in range(lc - 1):
                currx = int(curve[i].x)
                curry = int(curve[i].y)
                nextx = int(curve[i + 1].x)
                nexty = int(curve[i + 1].y)
                pygame.gfxdraw.line(self.screen, currx, curry, nextx, nexty, curve[0].color)
            # pygame.draw.aalines(self.screen, curve[0].color, False, pts)

        self.p.display(screen=self.screen)
        self.render_frame(offset=100)

    def loop(self):
        self.configure()
        self.initial_frame()

        pygame.display.flip()

        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.fill((255, 255, 255))
            self.render()
            pygame.display.flip()
