#!/usr/bin/env python3
import rospy
from accel_control import AccelControl


def main():

    rospy.init_node('accel_control', anonymous=True)
    AccelControl()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
