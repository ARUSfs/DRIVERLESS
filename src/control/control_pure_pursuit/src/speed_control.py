"""Script to develope PID controller to accelerator control.

@author: Mariano del Río
@date: 20220520
"""

import time
import rospy

# Constants
MAX_CMD = rospy.get_param('/control_pure_pursuit/max_cmd')  # maximum command

class PIDController():

    def __init__(self, Kp: float, Kd: float, Ki: float, time):
            
        self.kp = Kp
        self.kd = Kd
        self.ki = Ki

        self.Cp = 0
        self.Cd = 0
        self.Ci = 0

        self.time = time
        self.prevtime = self.time
        self.prev_error = 0

    def setKp(self, newKp: float):
        self.kp = newKp

    def setKd(self, newKd: float):
        self.kd = newKd

    def setKi(self, newKi: float):
        self.ki = newKi

    def setPrevError(self, prev: float):
        self.prev_error = prev

    def accelerator_control(self, current: float, target: float):

        error = target - current
        self.time = time.time()
        dt = self.time-self.prevtime

        # Calculate gains
        de = error-self.prev_error
        self.Cp = error
        self.Ci += error*dt
        self.Cd = 0

        if dt > 0:
            self.Cd = de/dt

        # Update time and error
        self.prevtime = self.time
        self.prev_error = error

        cmd = self.kp * self.Cp + self.ki * self.Ci + self.kd * self.Cd

        return cmd
