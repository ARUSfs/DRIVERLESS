"""Script to save estimated car state in certain time

@author: Mariano del Río
@date: 20220320
"""

import math


class State:

    def __init__(self, x: float, y: float, yaw: float, v: float, time,
                 WB: float):

        self.WB = WB
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.t = time
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))

    def update(self, x: float, y: float, yaw: float, v: float, t):

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.t = t
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x: float, point_y: float):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:
    """Class to save all estimated states.
    """

    def __init__(self):

        self.states = []

    def append(self, state: State):

        self.states.append(state)
