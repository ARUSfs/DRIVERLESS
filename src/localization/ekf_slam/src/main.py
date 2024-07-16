#!/usr/bin/env python3
import rospy
from ekf_slam import EKF_SLAM


def main():

    rospy.init_node('ekf_slam', anonymous=True)
    EKF_SLAM()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
