#!/usr/bin/env python3
import rospy
from skidpad_control import SkidpadControl


def main():

    rospy.init_node('skidpad_control', anonymous=True)
    SkidpadControl()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
