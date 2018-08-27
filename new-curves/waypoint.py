from numpy import ndarray, array as nparr
from math import cos, sin, radians
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

    def point(self) -> ndarray:
        return nparr(self.point)

    def first_derivative(self) -> ndarray:
        x = cos(self.radians)
        y = sin(self.radians)
        return nparr([x, y])

    def second_derivative(self, is_to_the_left: bool = False) -> ndarray:
        coeff = 1 if is_to_the_left else -1
        x = coeff * sin(self.radians)
        y = cos(self.radians)
        return nparr([x, y])