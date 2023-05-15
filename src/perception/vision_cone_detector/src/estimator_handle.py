#!/usr/bin/env python3
"""Script to execute the node visual perception node handle

@author: Jacobo Pindado (modified by Jorge Muñoz to add no homography topic)
@date: 20221113
"""
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
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
        self.pub_without_homography = rospy.Publisher("/vision_cone_detector/estimated_map_without_homography", Map, queue_size=10)

    def camera_callback(self, msg: Image):
        pub_map = Map()
        # The timestamp is the same as the camera's to ignore inference time when using
        # tf2 and other time dependent transforms.
        pub_map.header = Header()
        pub_map.header.stamp = msg.header.stamp
        pub_map.header.frame_id = "camera_2d"

        encoding = "passthrough" if camera["img_format"] == "RGB" else "rgb8"
        image = CvBridge().imgmsg_to_cv2(msg, desired_encoding=encoding)

        estimated_points_without_homography, estimated_points_with_homography = self.estimator.map_from_image(image)

        self._publish_points(estimated_points_without_homography, self.pub_without_homography, pub_map)
        self._publish_points(estimated_points_with_homography, self.pub, pub_map)

    def _publish_points(self, estimated_points, publisher, pub_map):
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
            
            cone.confidence = confidence
            pub_map.cones.append(cone)

        publisher.publish(pub_map)

