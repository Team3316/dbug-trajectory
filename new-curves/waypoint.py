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
        self.radians = radians(angle)

    def first_derivative(self, scale: float = 3) -> List[float]:
        x = cos(self.radians) * scale
        y = sin(self.radians) * scale
        return [x, y]

    def second_derivative(self) -> List[float]:
        x = -sin(self.radians)
        y = cos(self.radians)
        return [x, y]

    def distance_to(self, waypoint) -> float:
        dx = waypoint.point[0] - self.point[0]
        dy = waypoint.point[1] - self.point[1]
        return sqrt(dx ** 2 + dy ** 2)