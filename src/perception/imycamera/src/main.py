#!/usr/bin/env python3
"""Script to initialize the visual perception node

@author: Jacobo Pindado
@date 20221113
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header


if __name__ == '__main__':
    rospy.init_node('imycamera', anonymous=True)

    capture_device = rospy.get_param('~capture_device', 0)
    capture_fps = rospy.get_param('~capture_fps', 30.0)
    capture_width = rospy.get_param('~capture_width', 1280)
    capture_height = rospy.get_param('~capture_height', 960)
    camera_frame = rospy.get_param('~camera_frame', None)

    pub = rospy.Publisher('~image_raw', Image, queue_size=1)

    capture = cv2.VideoCapture(capture_device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
    capture.set(cv2.CAP_PROP_FPS, capture_fps)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    bridge = CvBridge()

    rate = rospy.Rate(capture_fps)
    try:
        header = Header()
        header.frame_id = camera_frame

        while not rospy.is_shutdown():
            while not capture.grab():
                continue

            _, image = capture.retrieve()
            if image is not None:
                header.stamp = rospy.get_rostime()
                image_msg = bridge.cv2_to_imgmsg(image,
                                                 encoding='passthrough',
                                                 header=header)

                pub.publish(image_msg)

            rate.sleep()

        # If we exit the while loop, release the camera from opencv
        capture.release()
    except rospy.ROSInterruptException:
        capture.release()
