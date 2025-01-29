#!/usr/bin/env python3
import rospy
from cones_detect import Video_stream


def main():
    rospy.init_node("cam_stream", anonymous=True)
    Video_stream().stream()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
