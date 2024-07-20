#Si salen picos muy similares podemos poner un umbral más bajo y filtrar posteriormente.

import rospy
import numpy as np
from scipy.signal import find_peaks
from std_msgs.msg import Float32, Float64
from common_msgs.msg import CarState, Controls

freq = 100

class Lap_counter:
    def __init__(self):
        self.lap_count = 0
        self.first_period = 0
        self.next_lap = 0
        self.angles = []
        self.yaw = 0
        self.mean = 0
        self.peaks = []
        self.vx = 0

        rospy.Subscriber('/first_lap_periode', Float32, self.callback, queue_size=10)
        rospy.Subscriber('/controls', Controls, self.angle_callback, queue_size=10)
        rospy.Subscriber('/car_state/state', CarState, self.yaw_callback, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.pubLapCount)

        self.pubLapCount = rospy.Publisher('/lap_count', Float32, queue_size=10)
        self.pubNextLap = rospy.Publisher('/next_lap', Float32, queue_size=10)

    def callback(self, msg: Float32):
        self.lap_count += 1
        rospy.logwarn(msg)
        self.first_period = msg.data
        rospy.Timer(rospy.Duration(msg.data/3.), self.update)

    def angle_callback(self, msg: Float32):
        if(self.vx > 0.1):
            self.angles.append(msg.steering*self.yaw)

    def yaw_callback(self, msg: Float64):
        rospy.logwarn(msg.vx)
        self.vx = msg.vx
        self.yaw = msg.yaw

    def update(self, event):
        rospy.logwarn('Updating')
        autocorr = np.correlate(self.angles[:int(np.round(self.first_period * freq))], self.angles, mode='full')
        self.peaks, _ = find_peaks(autocorr, height=np.max(autocorr) * 0.90)
        
        if len(self.peaks) > 0:
            self.peaks = np.append(self.peaks, np.zeros(9-len(self.peaks)))
            self.lap_count = len(self.peaks)
            self.mean = np.mean([self.peaks[i+1]-self.peaks[i] for i in range(len(self.peaks)-1)])
            self.next_lap = self.peaks[-1] + self.mean
        else:
            rospy.logwarn('Puta')

        if self.lap_count == 9:
            x2 = Float32()
            x2.data = self.next_lap
            self.pubNextLap.publish(x2)
    
    def pubLapCount(self, event):
        x = Float32()
        x.data = self.lap_count
        self.pubLapCount.publish(x)

        
