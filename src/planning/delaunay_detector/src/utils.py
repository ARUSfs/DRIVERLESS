"""
Script to implement utils funcions.

@author: Mariano del Río
@date: 20220320
"""

import numpy as np
import math
from scipy.interpolate import splprep, splev


def spline(trajectory: np.ndarray):

    tck, u = splprep(trajectory.T, s=0.0, k=3)
    x = np.linspace(u.min(), u.max(), 100)
    x, y = splev(x, tck)

    return x, y


def distance2D(p1: np.ndarray, p2: np.ndarray):

    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def get_cos(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):

    v1 = p2-p1
    v2 = p3-p2

    v1_mod = np.linalg.norm(v1)
    v2_mod = np.linalg.norm(v2)

    scalar_product = v1.dot(v2.T)
    cos = scalar_product/(v1_mod*v2_mod)

    return (-1)*cos


def midpoint(p1: np.ndarray, p2: np.ndarray):
    return np.array([(p1[0]+p2[0])/2, (p1[1]+p2[1])/2])
