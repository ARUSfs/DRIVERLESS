#!/bin/env python3
"""
Script that will automatically output the homography matrix to be used by the estimator. Given
the following cone shape:
               b
              y y
             b y b
, where each cone is separated to the closest one by CONE_DISTANCE meters.

The returned transformation matrix will 

@author: Jacobo Pindado
@date: 20221225
"""
import sys
import argparse
from math import sqrt

import cv2
import yaml
import numpy as np

from detector import YoloDetector, darknet
from keypoints.keypoint_detector import keypoint_detector

CONE_DISTANCE = 2  # m

real_points = np.array([((j - i/2)*CONE_DISTANCE, -i*CONE_DISTANCE*sqrt(3)/2, 0.32)
                        for i in range(3)
                        for j in range(i+1)], dtype=np.float32)

parser = argparse.ArgumentParser(
    prog = 'Auto Calibration',
    description = 'Will take an image or camera device and extract the \
                   homography and transformation matrix'
)

parser.add_argument('config',
                    help='Path to node config file with Camera Matrix \
                          and Distortion Coefficients')
parser.add_argument('--image',
                    nargs='?',
                    help='Path to image')
parser.add_argument('--video',
                    nargs='?',
                    help='Path to video or device. \
                    First captured image will be used')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        parser.print_usage()
        sys.exit(1)

    options = parser.parse_args()

    if options.video is None and options.image is None:
        print('No input image/video was given, so nothing to be calibrated')
        sys.exit(2)
    elif options.image is not None:
        img = cv2.imread(options.image)
    else:
        cap = cv2.VideoCapture(options.video)
        if cap.grab():
            img = cap.retrieve()[1]
        else:
            print('OpenCV returned false when trying to read video. Is the path OK?')
            sys.exit(2)

    with open(options.config, 'r') as stream:
        try:
            yaml_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            sys.exit(3)

    camera_mat = yaml_file['camera']['camera_mat']
    camera_mat = np.array(camera_mat).reshape((3, 3))
    dist = np.array(yaml_file['camera']['dist_coefs'])
    (width, height) = tuple(yaml_file['camera']['image_size'])
    yolo_cfg = yaml_file['yolo']['cfg']
    yolo_data = yaml_file['yolo']['data']
    yolo_weights = yaml_file['yolo']['weights']
    kpt_num = yaml_file['kpt']['num']
    kpt_weights = yaml_file['kpt']['weights']
    new_camera_mat = cv2.getOptimalNewCameraMatrix(camera_mat, dist, (width, height), 0)[0]

    if img.shape != (height, width, 3):
        img = cv2.resize(img, (width, height))
    img = cv2.undistort(img, camera_mat,
                        dist, None,
                        new_camera_mat)

    yolo_detector = YoloDetector(yolo_cfg, yolo_data, yolo_weights)
    kpt_detector = keypoint_detector(kpt_num, kpt_weights)
    detections = yolo_detector.predict_from_image(img)

    blue_cones = list()
    yellow_cones = list()

    for i, (class_name, confidence, bbox) in enumerate(detections):
        xmin, ymin, xmax, ymax = darknet.bbox2points(bbox)
        xmin = max(0, xmin)
        ymin = max(0, ymin)
        xmax = min(xmax, img.shape[1] - 1)
        ymax = min(ymax, img.shape[0] - 1)
        cone_bbox_img = img[ymin:ymax, xmin:xmax]

        # keypoints relative to bounding box
        kpts_relative = kpt_detector.get_keypoint_from_image(cone_bbox_img)

        vertex = tuple(kpts_relative[0] + np.array([xmin, ymin]))

        if class_name == 'blue_cone':
            blue_cones.append(vertex)
        else:
            yellow_cones.append(vertex)

    img_points = np.empty((6, 2))

    v = min(blue_cones, key=lambda a: a[1])
    img_points[0, :] = np.array(v)
    blue_cones.remove(v)
    v = min(blue_cones, key=lambda a: a[0])
    img_points[3, :] = np.array(v)
    blue_cones.remove(v)
    img_points[5, :] = np.array(blue_cones.pop(0))

    v = max(yellow_cones, key=lambda a: a[1])
    img_points[4, :] = np.array(v)
    yellow_cones.remove(v)
    v = min(yellow_cones, key=lambda a: a[0])
    img_points[1, :] = np.array(v)
    yellow_cones.remove(v)
    img_points[2, :] = np.array(yellow_cones.pop(0))

    _, rvec, tvec = cv2.solvePnP(real_points, img_points, camera_mat, dist)

    print(f'Rotation Vector: \n{rvec}')
    print(f'Translation Vector: \n{tvec}')

    rmat = cv2.Rodrigues(rvec)[0]
    pmat = rmat
    pmat[:, 2] = pmat[:, 2]*0.32 + tvec.flatten()

    H = new_camera_mat @ pmat
    print(f'H mat: (u, v, 1) = H @ (X, Y, 1) -> \n{H}')
    print(f'H\' mat: H\' @ (u, v, 1) = (X, Y, 1) -> \n{np.linalg.inv(H)}')
