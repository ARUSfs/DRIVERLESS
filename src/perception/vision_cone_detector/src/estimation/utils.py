"""
Auxiliary methods for distance calculations, distortion and calibration of cameras.

@author: Jacobo Pindado
@date: 20220223
"""
import cv2
import numpy as np


def undistort_pt(point, dist, iter_num=3):
    """Aplies undistortion of a point given it's distortion coeficients.
    The corresponding cv2 method is apparently wrongly implemented,
    so here is the correct one. This method asumes that the point in pixels
    has already been multiplied by the camera matrix.

    :param point: array-like with [x,y,1] coordinates of point in image.
    :param dist: radial and tangential distortion coeficient list
    :param iter_num: number of iterations to run algorithm
    :return: coordinates of undistorted point
    """
    if not isinstance(point, np.ndarray):
        point = np.array(point)

    k1, k2, p1, p2, k3 = dist

    x, y = point[:2].astype(float)
    x0 = x
    y0 = y

    for _ in range(iter_num):
        r2 = x ** 2 + y ** 2
        k_inv = 1 / (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3)
        delta_x = 2 * p1 * x*y + p2 * (r2 + 2 * x**2)
        delta_y = p1 * (r2 + 2 * y**2) + 2 * p2 * x*y
        x = (x0 - delta_x) * k_inv
        y = (y0 - delta_y) * k_inv

    return np.array([x, y, 1])


def undistort_px(point, mtx, dist, iter_num=3):
    """Aplies undistortion of a point given it's distortion coeficients.
    Unlike undistort_pt, which requires the point to already be multiplied
    by the camera matrix, this function takes the mtx too.

    :param point: array-like with x,y coordinates of point in image
    :param mtx: camera matrix with intrinsic parameters
    :param dist: radial and tangential distortion coeficient list
    :param iter_num: number of iterations to run algorithm
    :return: coordinates of undistorted point
    """
    if not isinstance(point, np.ndarray):
        point = np.array(point)

    k1, k2, p1, p2, k3 = dist

    x, y, _ = px2units(point, mtx)
    x0 = x
    y0 = y
    for _ in range(iter_num):
        r2 = x ** 2 + y ** 2
        k_inv = 1 / (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3)
        delta_x = 2 * p1 * x*y + p2 * (r2 + 2 * x**2)
        delta_y = p1 * (r2 + 2 * y**2) + 2 * p2 * x*y
        x = (x0 - delta_x) * k_inv
        y = (y0 - delta_y) * k_inv
    return units2px(point, mtx)


def px2units(point, mtx):
    """Equivalent to multiplying point and camera matrix.

    :param point: array-like with x,y coordinates of point in image
    :param mtx: camera matrix with intrinsic parameters
    :return: point in camera-matrix units [x',y',1]
    """
    x = (point[0] - mtx[2, 0]) / mtx[0, 0]
    y = (point[1] - mtx[2, 1]) / mtx[1, 1]
    return np.array([x, y, 1])


def units2px(point, mtx):
    """Equivalent to multiplying point and inverse camera matrix.

    :param point: array-like with [x,y] coordinates of point in image
    :param mtx: camera matrix with intrinsic parameters
    :return: point in camera-matrix units [x',y',1]
    """
    x = point[0]*mtx[0, 0] + mtx[2, 0]
    y = point[1]*mtx[1, 1] + mtx[2, 1]
    return np.array([x, y])


def invert_camera_mtx(mtx):
    inverse_mtx = mtx.copy()
    inverse_mtx[0][0] = 1/mtx[0][0]
    inverse_mtx[1][1] = 1/mtx[1][1]
    inverse_mtx[2][0] = -mtx[2][0]/mtx[0][0]
    inverse_mtx[2][1] = -mtx[2][1]/mtx[1][1]

    return inverse_mtx


def calibrate_from_chessboard(directory, progress_bar=False):
    """Obtain camera matrix and distortion parameters from chessboard images.

    :param directory: path to directory containing .jpg or .png images of the chessboard.
    :return: mtx - camera matrix, dist - distortion parameters.
    """
    from glob import glob
    if progress_bar:
        from tqdm import tqdm

    if directory[-1] != "/":
        directory += "/"

    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((7*7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:7].T.reshape(-1, 2)
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob(directory + '*.jpg') + glob(directory + '*.png')

    if progress_bar:
        images = tqdm(images)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7, 7), cv2.CALIB_CB_ADAPTIVE_THRESH)
        if ret:
            objpoints.append(objp)

    try:
        ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints,
                                                   imgpoints,
                                                   gray.shape[::-1],
                                                   None,
                                                   None)
        return mtx, dist
    except cv2.Exception:
        raise Exception("None of the provided images are valid. Check that the full 8x8 grid is \
                         visible with high enough contrast")
