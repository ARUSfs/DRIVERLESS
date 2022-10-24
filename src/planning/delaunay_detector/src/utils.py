"""Script to implement utils funcions.

@author: Mariano del Río
@date: 20220320
"""

import rospy
import numpy as np
import math
from scipy.interpolate import splprep, splev

NUM_POINTS = rospy.get_param('/delaunay_detector/NUM_POINTS')
DEGREE = rospy.get_param('/delaunay_detector/DEGREE')


def spline(trajectory: np.ndarray):

    tck, u = splprep(trajectory.T, s=0.0, k=DEGREE)
    x = np.linspace(u.min(), u.max(), NUM_POINTS)
    x, y = splev(x, tck)
    return x, y


def distance2D(p1: np.ndarray, p2: np.ndarray):
    return np.linalg.norm(p1-p2)


def get_cos(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):

    v1 = p2 - p1
    v2 = p3 - p3
    v1_mod = np.linalg.norm(v1)
    v2_mod = np.linalg.norm(v2)
    cos = v1.dot(v2.T)/(v1_mod*v2_mod)
    return (-1)*cos


def midpoint(p1: np.ndarray, p2: np.ndarray):
    return (p1+p2)/2
