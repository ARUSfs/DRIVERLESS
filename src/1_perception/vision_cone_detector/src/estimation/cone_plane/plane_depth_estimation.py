"""
Methods to estimate cone positions using the 'plane' method, taking advantage of
knowing cone's heights.

@author: Jacobo Pindado
@date: 20220223
"""

import numpy as np
from ..utils import undistort_pt, px2units


def get_point_coordinates(point, plane, mtx, dist, rotmtx, tvec, inverse_mtx=None):
    """Obtains the point's 3D coordinates by finding intersection of projecting line and plane.

    :param point: array-like with x,y coordinates of point in image.
    :param plane: array-like [a1, a2, a3, b] where a1x+a2y+a3z=b is the plane's equation.
    :param mtx: camera matrix with intrinsic parameters
    :param dist: radial and tangential distortion coeficient list
    :param rotmtx: camera's rotation matrix.
    :param tvec: camera's translation vector relative to reference frame origin.
    :return: point's coordinates en 3D. Reference frame will depend on rotmtx and tvec.
    """
    if not isinstance(point, np.ndarray):
        point = np.array(point)

    point = px2units(point, mtx)  # Multiplies camera matrix and point.
    point = undistort_pt(point, dist)

    # We apply inverse of rotation matrix. We take advantage of the fact that rotmat^-1 = rotmat^t.
    # vect will represent the direction vector of the line that
    # goes through tvec and the real point.
    vect = rotmtx.T @ point

    # We set up the linear equations of the system. One will correspond to the plane while the
    # other two to the line going through tvec and direction vect.
    A = np.concatenate(([[vect[1], -vect[0],        0]],
                        [[vect[2],        0, -vect[0]]],
                        [plane[0:3]]),
                       axis=1)

    b = np.array([float(tvec[0]*vect[1]-tvec[1]*vect[0]),
                  float(tvec[0]*vect[2]-tvec[2]*vect[0]),
                  plane[3]])

    final_point = np.linalg.solve(A, b)

    return final_point
