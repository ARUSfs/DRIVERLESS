"""Script to implement main class to control planning and
car controls.

@author: Mariano del Río
@date: 20220320
"""

from pure_pursuit import pure_pursuit_control, TargetCourse
from car_state import State, States
from speed_control import PIDController
import rospy

# Constants
K = rospy.get_param('/control_pure_pursuit/K')  # look forward gain
LFC = rospy.get_param('/control_pure_pursuit/LFC')  # [m] look-ahead distance
KP = rospy.get_param('/control_pure_pursuit/KP')  # speed proportional gain
KI = rospy.get_param('/control_pure_pursuit/KI')    # speed integrative gain
KD = rospy.get_param('/control_pure_pursuit/KD')    # speed derivative gain
WB = rospy.get_param('/control_pure_pursuit/WB')  # [m] wheel base of vehicle
TARGET_SPEED = rospy.get_param('/control_pure_pursuit/TARGET_SPEED')  # [m/s]


class ControlCar():
    """Main class to save and update trajectory and calculate
    command controls via pure pursuit and PID controllers.
    """

    def __init__(self, time):

        origin = [0.0, 0.0]
        yaw = 0
        v = 0
        self.state = State(origin[0], origin[1], yaw, v, time, WB)
        self.previous_states = States()
        self.previous_states.append(self.state)

        self.target_course = TargetCourse(K, LFC)
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

    def get_cmd(self):
        
        ai = self.pid.accelerator_control(self.state.v, TARGET_SPEED)
        # ai = 0.2
        di, ind = pure_pursuit_control(self.state, self.target_course,
                                       self.target_ind, WB)
        self.target_ind = ind

        return ai, di
