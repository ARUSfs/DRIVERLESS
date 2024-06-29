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
import rospkg

if __name__ == '__main__':
    rospy.init_node('imycamera', anonymous=True)

    rospack = rospkg.RosPack()
    path = rospack.get_path('imycamera')
    capture_device = rospy.get_param('~capture_device', 0)
    capture_fps = rospy.get_param('~capture_fps', 30.0)
    capture_width = rospy.get_param('~capture_width', 1280)
    capture_height = rospy.get_param('~capture_height', 960)
    camera_frame = rospy.get_param('~camera_frame', None)

    pub = rospy.Publisher('~image_raw', Image, queue_size=1)

    capture = cv2.VideoCapture(capture_device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    capture.set(cv2.CAP_PROP_FPS, capture_fps)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    bridge = CvBridge()

    out = cv2.VideoWriter(path+'/vsv.mp4', cv2.VideoWriter_fourcc(*'mp4v'), capture_fps, (capture_width, capture_height))
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
                img = cv2.resize(image, (capture_width, capture_height))
                out.write(img)
                image_msg = bridge.cv2_to_imgmsg(image,
                                                 encoding='bgr8',
                                                 header=header)
                rospy.logwarn(img.shape)
                pub.publish(image_msg)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            rate.sleep()

        # If we exit the while loop, release the camera from opencv
        capture.release()
        out.release()
    except rospy.ROSInterruptException:
        out.release()
        capture.release()
