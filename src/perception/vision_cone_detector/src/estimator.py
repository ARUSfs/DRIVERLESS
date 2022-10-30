#!/usr/bin/env python3
"""
Class that will be used by rospy to pass each image through all the
perception pipeline. The estimator will take an image as an input,
and will output an estimation of the position of each cone relative
to the car.

@author: Jacobo Pindado
@date: 20221030
"""
from dataclasses import dataclass

import cv2
import numpy as np

from detector import YoloDetector
from keypoints import keypoint_detector


class Estimator:
    def __init__(self,
                 yolo_cfg: Union[str, bytes, os.PathLike],
                 yolo_data: Union[str, bytes, os.PathLike],
                 yolo_weights: Union[str, bytes, os.PathLike],
                 kpt_num: int,
                 kpt_weights: Union[str, bytes, os.PathLike],
                 kpt_img_size: Tuple[int, int],
                 cameras: dict[str, CameraConfig]):

        self.yolo_detector = YoloDetector(yolo_cfg, yolo_data, yolo_weights)
        self.kpt_detector = keypoint_detector(kpt_num, kpt_weights, kpt_img_size)
        self.cameras = cameras

    def get_from_identifier(self, camera_identifier: str, img: np.ndarray):
        cam_conf = self.cameras[camera_identifier]
        if cam_conf.img_format == 'RGB':
            pass
        elif cam_conf.img_format == 'BRG':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            raise NotImplementedError("At this moment only RGB and BRG formats\
                                      are supported. Inputed image format is {}."\
                                      .format(cam_conf.img_format))

        detections = self.yolo_detector.predict_from_image(img)

        for class_name, confidence, bbox in detections:
            


@dataclass
class CameraConfig:
    camera_mat: np.ndarray
    dist_coefs: np.ndarray
    homography_mat: np.ndarray
    img_format: str
