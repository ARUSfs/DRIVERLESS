#!/usr/bin/env python3
import rospy
from stanley_control import StanleyControl


def main():

    rospy.init_node('stanley_control', anonymous=True)
    StanleyControl()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
