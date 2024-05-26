"""Script to implement pure pursuit algorithm.

@author: Mariano del Río
@date: 20220320
"""

import numpy as np
import math
from car_state import State
import rospy

MIN_VEL =  rospy.get_param('/control_pure_pursuit/min_vel') # minimum velocity to start steering

class TargetCourse:
    """Class to choose pursuit point of trajectory.
    """

    def __init__(self, k: float, Lfc: float):

        self.cx = []
        self.cy = []  # Points from planning

        self.k = k
        self.Lfc = Lfc
        self.Lf = Lfc
        self.old_nearest_point_index = None

    def update_route(self, new_x: list, new_y: list):

        self.cx = new_x
        self.cy = new_y
        self.old_nearest_point_index = None

    def search_target_index(self, state: State):

        if len(self.cx)==0:
            return -1, self.Lf

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            dist_this_index = state.calc_distance(self.cx[ind],
                                                  self.cy[ind])
            while (ind+1<len(self.cx)):
                dist_next_index = state.calc_distance(self.cx[ind + 1],
                                                      self.cy[ind + 1])

                if dist_this_index < dist_next_index:
                    break

                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                dist_this_index = dist_next_index

            self.old_nearest_point_index = ind

        self.Lf = self.k * state.v + self.Lfc  # update look ahead distance

        # search look ahead target point index
        while self.Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, self.Lf


def pure_pursuit_control(state: State, trajectory: list, pind: int, WB: float, DT: float):

    ind, Lf = trajectory.search_target_index(state)

    if ind == -1 or state.v < MIN_VEL:
        delta = state.prev_delta
    else:
        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x)

        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        delta = max(-19.9,min(math.degrees(delta),19.9))

        delta = max(state.prev_delta-DT,min(delta,state.prev_delta+DT))

        state.prev_delta = delta

    return delta, ind
