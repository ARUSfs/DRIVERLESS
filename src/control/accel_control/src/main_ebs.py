#!/usr/bin/env python3
import rospy
from ebs_test_control import EBSTestControl


def main():

    rospy.init_node('ebs_test_control', anonymous=True)
    EBSTestControl()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
