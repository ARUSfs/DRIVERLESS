#!/usr/bin/env python3
"""Script to execute the node visual perception node handle

@author: Jacobo Pindado
@date 20221113
"""
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from estimator import Estimator, CameraConfig
from common_msgs.msg import Cone, Map

yolo_cfg = rospy.get_param("/vision_cone_detector/yolo/cfg")
yolo_data = rospy.get_param("/vision_cone_detector/yolo/data")
yolo_weights = rospy.get_param("/vision_cone_detector/yolo/weights")
kpt_num = rospy.get_param("/vision_cone_detector/kpt/num")
kpt_weights = rospy.get_param("/vision_cone_detector/kpt/weights")
kpt_img_size = tuple(rospy.get_param("/vision_cone_detector/kpt/img_size"))
camera = rospy.get_param("/vision_cone_detector/camera")


class EstimatorHandle():

    def __init__(self):
        self.estimator = Estimator(yolo_cfg,
                                   yolo_data,
                                   yolo_weights,
                                   kpt_num,
                                   kpt_weights,
                                   kpt_img_size,
                                   CameraConfig.from_param(camera))

        rospy.Subscriber(camera["topic"], Image, self.camera_callback, queue_size=1)

        self.pub = rospy.Publisher("/vision_cone_detector/estimated_map", Map, queue_size=10)

    def camera_callback(self, msg: Image):
        rospy.logerr("received========================================")
        encoding = "passthrough" if camera["img_format"] == "RGB" else "rgb8"
        image = CvBridge().imgmsg_to_cv2(msg, desired_encoding=encoding)
        if image.shape != (1088, 1920, 3):
            image = cv2.resize(image, (1920, 1088))

        estimated_points = self.estimator.map_from_image(image)

        pub_map = Map()
        pub_map.cones = list()

        for color, confidence, (x, y) in estimated_points:
            cone = Cone()
            cone.position = Point()
            cone.position.x = x
            cone.position.y = y
            cone.position.z = 0

            if color == "yellow_cone":
                cone.color = 'y'
            elif color == "blue_cone":
                cone.color = 'b'
            elif color == "orange_cone":
                cone.color = 'o'
            elif color == "large_orange_cone":
                cone.color = 'O'
            else:
                cone.color = 'u'

            cone.probability = confidence
            pub_map.cones.append(cone)

        self.pub.publish(pub_map)
        rospy.logerr("sent////////////////////////////////////////////////////////")
