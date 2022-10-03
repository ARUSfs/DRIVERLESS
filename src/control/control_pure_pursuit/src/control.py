"""
Script to implement main class to control planning and
car controls.

@author: Mariano del Río
@date: 20220320
"""

from pure_pursuit import pure_pursuit_control, targetCourse
from car_state import State, States
from speed_control import PIDController

# Constants
K = 0.30  # look forward gain
LFC = 8.55  # [m] look-ahead distance
KP = 1.0  # speed proportional gain
KI = 0.2    # speed integrative gain
KD = 0.2    # speed derivative gain
WB = 1.5  # [m] wheel base of vehicle
TARGET_SPEED = 10.0 / 3.6  # [m/s]


class controlCar():
    """
    Main class to save and update trajectory and calculate
    command controls via pure pursuit and PID controllers.
    """

    def __init__(self, time):

        origin = [0.0, 0.0]
        yaw = 0
        v = 0
        self.state = State(origin[0], origin[1], yaw, v, time, WB)
        self.previous_states = States()
        self.previous_states.append(self.state)

        self.target_course = targetCourse(K, LFC)
        self.target_ind = None
        self.existPath = False  # True if there is trajectory

        self.pid = PIDController(KP, KD, KI, time)

    def update_trajectory(self, newx: list, newy: list):

        self.target_course.update_route(newx, newy)
        self.target_ind, _ = self.target_course.search_target_index(self.state)
        self.existPath = True

    def update_state(self, time, x: float, y: float, yaw: float, v: float):

        self.state.update(x, y, yaw, v, time)
        self.previous_states.append(self.state)

    def getCmd(self):

        ai = self.pid.accelerator_control(self.state.v, TARGET_SPEED)
        di, ind = pure_pursuit_control(self.state, self.target_course,
                                       self.target_ind, WB)
        self.target_ind = ind

        return ai, di
