#!/usr/bin/env python3
import rospy
from cones_detect import Cone_detect


def main():
    rospy.init_node("cam_detect", anonymous=True)
    Cone_detect()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
