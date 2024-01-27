"""Script to save estimated car state in certain time

@author: Mariano del Río
@date: 20220320
"""

import math


class State:

    def __init__(self, v: float, prev_delta: float, time,
                 WB: float):

        self.WB = WB
        self.v = v
        self.prev_delta = prev_delta
        self.t = time
        self.rear_x = -(self.WB / 2)
        self.rear_y = 0

    def update(self, v: float, t):

        self.v = v
        self.t = t
        self.rear_x = -(self.WB / 2)
        self.rear_y = 0

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
