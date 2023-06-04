#!/usr/bin/env python3
"""Script to initialize the MPC code

@author: Jacobo Pindado Perea
@author: Jacobo Pindado Perea
@date: 20230516
"""

import rospy
from control_handle import ControlHandle


def main():

    rospy.init_node('control_pure_pursuit', anonymous=True)
    ControlHandle()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
