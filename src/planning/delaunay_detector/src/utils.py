"""Script to implement utils funcions.

@author: Mariano del Río
@date: 20220320
"""

import rospy
import numpy as np
import scipy.interpolate as si

NUM_POINTS = rospy.get_param('/delaunay_detector/NUM_POINTS')
DEGREE = rospy.get_param('/delaunay_detector/DEGREE')


def spline(trajectory: np.ndarray):

    x = trajectory[:, 0]
    y = trajectory[:, 1]

    n = len(x)
    if n > 3:
        knotspace = range(n)

        knots = si.InterpolatedUnivariateSpline(knotspace, knotspace, k=DEGREE).get_knots()
        knots_full = np.concatenate(([knots[0]]*DEGREE, knots, [knots[-1]]*DEGREE))
        tckX = knots_full, x, DEGREE
        tckY = knots_full, y, DEGREE
        splineX = si.UnivariateSpline._from_tck(tckX)
        splineY = si.UnivariateSpline._from_tck(tckY)

        tp = np.linspace(knotspace[0], knotspace[-1], NUM_POINTS)
        xp = splineX(tp)
        yp = splineY(tp)
    else:
        xp, yp = x, y

    return zip(xp, yp)


def distance2D(p1: tuple, p2: tuple):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def get_cos(p1: tuple, p2: tuple, p3: tuple):
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p2)
    v1_mod = np.linalg.norm(v1)
    v2_mod = np.linalg.norm(v2)
    cos = v1.dot(v2.T)/(v1_mod*v2_mod)
    return (-1)*cos


def midpoint(p1: tuple, p2: tuple):
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
