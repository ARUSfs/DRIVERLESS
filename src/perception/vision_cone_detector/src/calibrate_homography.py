"""
Script that will automatically output the homography matrix to be used by the estimator. Given
the following cone shape:
               b
              y y
             y y y
            b b b b
, where each cone is separated to the closest one by CONE_DISTANCE meters.

@author: Jacobo Pindado
@date: 20221225
"""
import sys
import argparse

import cv2
import yaml
import numpy as np
from scipy.spatial import Delaunay

from detector import YoloDetector
from keypoints.keypoint_detector import keypoint_detector

CONE_DISTANCE = 2  # m

parser = argparse.ArgumentParser(
    prog = 'Auto Calibration',
    description = 'Will take an image or camera device and extract the \
                   homography and transformation matrix'
)

parser.add_argument('config',
                    help='Path to node config file with Camera Matrix \
                          and Distortion Coefficients')
parser.add_argument('--image',
                    nargs=1,
                    help='Path to image')
parser.add_argument('--video',
                    nargs=1,
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

    with open(options.config, 'r') as stream:
        try:
            yaml_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            sys.exit(3)

    camera_mat = yaml_file['camera']['camera_mat']
    camera_mat = np.array(camera_mat).reshape((3,3))
    dist = np.array(yaml_file['camera']['dist_coefs'])
    (width, height) = tuple(yaml_file['camera']['image_size'])
    new_camera_mat = cv2.getOptimalNewCameraMatrix(camera_mat, dist, (width, height), 0)[0]



    if img.shape != (1088, 1920, 3):
        img = cv2.resize(img, (1920, 1088))
    img = cv2.undistort(img, self.cam_conf.camera_mat,
                        dist, None,
                        new_camera_mat)

    yolo_detector = YoloDetector()
    detections = yolo_detector.predict_from_image(img)

    vertex_array = np.empty((3, len(detections)))
    vertex_array[2, :] = np.ones(len(detections))
    vertex_properties = list()

    for i, (class_name, confidence, bbox) in enumerate(detections):
        xmin, ymin, xmax, ymax = darknet.bbox2points(bbox)
        # Due to rounding errors, boxes may fall outside the image
        xmin = max(0, xmin)
        ymin = max(0, ymin)
        xmax = min(xmax, img.shape[1] - 1)
        ymax = min(ymax, img.shape[0] - 1)
        cone_bbox_img = img[ymin:ymax, xmin:xmax]

        # keypoints relative to bounding box
        kpts_relative = self.kpt_detector.get_keypoint_from_image(cone_bbox_img)

        # Right now, we only apply homography to the vertex of the cone. Detecting 7 kpts
        # is unnecesary, but is left for future use. We also transform from relative to
        # absolute to whole image.
        vertex_array[:2, i] = kpts_relative[0] + np.array([xmin, ymin])

        vertex_properties.append((class_name, float(confidence)/100))

    vertex_array = self.cam_conf.transform_matrix @ vertex_array
    vertex_array /= vertex_array[2, :]  # We normalize the affine point

    return list((class_name, conf, (p[0], p[1])) for (class_name, conf), p
                in zip(vertex_properties, vertex_array.T))
