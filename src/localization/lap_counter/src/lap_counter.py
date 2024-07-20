#Si salen picos muy similares podemos poner un umbral más bajo y filtrar posteriormente.

import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from std_msgs.msg import Float32, Float64

freq = 100

class Lap_counter:
    def __init__(self):
        self.lap_count = 0
        self.first_period = 0
        self.next_lap = 0
        self.angles = []
        self.yaws = []
        self.mean = 0
        self.peaks = []

        rospy.Subscriber('/first_lap_periode', Float32, self.callbcak, queue_size=10)
        rospy.Subscriber('/controls/steering', Float32, self.angle_callback, queue_size=10)
        rospy.Subscriber('/car_state/state/yaw', Float64, self.yaw_callback, queue_size=10)

    def callback(self, msg: Float32):
        self.first_period = msg.data
        rospy.Timer(rospy.Duration(msg.data/3.), self.update)

    def angle_callback(self, msg: Float32):
        self.angles.append(msg.data)

    def yaw_callback(self, msg: Float64):
        self.yaws.append(msg.data)

    def update(self, event):
        autocorr = np.correlate(self.angles[:self.first_period * freq], self.angles, mode='full')
        self.peaks, _ = find_peaks(autocorr, height=np.max(autocorr) * 0.90)
        self.lap_count = len(self.peaks)
        self.mean = np.mean([self.peaks[i+1]-self.peaks[i] for i in range(len(self.peaks)-1)])
        self.next_lap = self.peaks[-1] + self.mean
