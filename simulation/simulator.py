import pygame
import pygame.gfxdraw

from .fps_counter import FpsCounter
from .particle import Particle
from typing import Tuple, List

class DBugSimulator:
    def __init__(self, graph_dimensions: Tuple[int, int], border: int, name: str):
        win_dimensions = (graph_dimensions[0] + border, graph_dimensions[1] + border)

        self.name = name
        self.screen = pygame.display.set_mode(win_dimensions)
        self.running = False
        self.dims = win_dimensions
        self.border = border

        self.counter = FpsCounter()

        self.curves = []
        self.timer = 0

    def configure(self):
        pygame.init()
        pygame.display.set_caption('Trajectory: %s' % self.name)

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
                pygame.gfxdraw.line(
                    self.screen,
                    int(curve[i].x), int(curve[i].y),
                    int(curve[i + 1].x), int(curve[i + 1].y),
                    curve[0].color
                )

        self.render_frame(offset=(2 * self.border))

    def loop(self):
        self.configure()

        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.fill((255, 255, 255))
            self.render()
            pygame.display.flip()
            self.counter.nexttick()
