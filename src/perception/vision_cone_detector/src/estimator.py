#!/usr/bin/env python3
"""
Class that will be used by rospy to pass each image through all the
perception pipeline. The estimator will take an image as an input,
and will output an estimation of the position of each cone relative
to the car.

@author: Jacobo Pindado
@date: 20221030
"""
import os
from typing import Union, List, Tuple
from dataclasses import dataclass

import cv2
import numpy as np

from detector.utils import darknet
from detector import YoloDetector
from keypoints.keypoint_detector import keypoint_detector


@dataclass
class CameraConfig:
    camera_mat: np.ndarray
    dist_coefs: np.ndarray
    homography_mat: np.ndarray
    img_format: str

    def from_param(cam_info):
        camera_mat = np.array(cam_info["camera_mat"]).reshape((3, 3))
        dist_coefs = np.array(cam_info["dist_coefs"])
        homography_mat = np.array(cam_info["homography_mat"]).reshape((3, 3))
        conf =  CameraConfig(camera_mat,
                             dist_coefs,
                             homography_mat,
                             cam_info["img_format"])
        return conf


class Estimator:
    def __init__(self,
                 yolo_cfg: Union[str, bytes, os.PathLike],
                 yolo_data: Union[str, bytes, os.PathLike],
                 yolo_weights: Union[str, bytes, os.PathLike],
                 kpt_num: int,
                 kpt_weights: Union[str, bytes, os.PathLike],
                 kpt_img_size: Tuple[int, int],
                 camera: CameraConfig):

        self.yolo_detector = YoloDetector(yolo_cfg, yolo_data, yolo_weights)
        self.kpt_detector = keypoint_detector(kpt_num, kpt_weights, kpt_img_size)
        self.cam_conf = camera

    def map_from_image(self,
                       img: np.ndarray) -> List[Tuple[str, float, Tuple[float, float]]]:

        detections = self.yolo_detector.predict_from_image(img)

        map_list = list()

        for class_name, confidence, bbox in detections:
            xmin, ymin, xmax, ymax = darknet.bbox2points(bbox)
            # Due to ronding errors, boxes may fall outside the image
            xmin = max(0, xmin)
            ymin = max(0, ymin)
            xmax = min(xmax, img.shape[1] - 1)
            ymax = min(ymax, img.shape[0] - 1)
            cone_bbox_img = img[ymin:ymax, xmin:xmax]

            # keypoints relative to bounding box
            kpts_relative = self.kpt_detector.get_keypoint_from_image(cone_bbox_img)

            # keypoints relative to whole image
            kpts_absolute = kpts_relative + np.array([[xmin, ymin]], dtype=np.float32)

            # Right now, we only apply homography to the vertex of the cone. Detecting 7 kpts
            # is unnecesary, but is left for future use.
            vertex_point = kpts_absolute[0, :]

            undistorted_vertex_point = cv2.undistortPoints(vertex_point.T,
                                                           self.cam_conf.camera_mat,
                                                           self.cam_conf.dist_coefs)

            map_point = self.cam_conf.homography_mat @ np.append(undistorted_vertex_point.T, 1)

            map_point = map_point / map_point[2]  # We normalize the affine point

            map_list.append((class_name, float(confidence), (map_point[0], map_point[1])))
        return map_list
