from math import cos, sin, radians, sqrt
from typing import List
from utils import Point


class Waypoint:
    """
    A class representing a path's waypoint
    """

    def __init__(self, point: Point, angle: float, time: float):
        """
        Initialize a new Waypoint object.
        :param point: The point in R^2 where the waypoint lays
        :param angle: The robot's heading angle when in the waypoint
        :param time: The time the robot should be on the waypoint
        """
        self.point = point
        self.angle = angle
        self.time = time

    def first_derivative(self, velocity: bool = False, scale: float = 1.5) -> List[float]:
        rads = radians(90 - self.angle if not velocity else self.angle)
        x = cos(rads) * scale
        y = sin(rads) * scale
        return [x, y]

    def second_derivative(self, scale: float = 0.15) -> List[float]:
        rads = radians(90 - self.angle)
        x = -sin(rads) * scale
        y = cos(rads) * scale
        return [x, y]

    def distance_to(self, waypoint) -> float:
        dx = waypoint.point[0] - self.point[0]
        dy = waypoint.point[1] - self.point[1]
        return sqrt(dx ** 2 + dy ** 2)
