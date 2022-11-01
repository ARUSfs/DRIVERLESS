#!/usr/bin/env python
"""Script to initialize control node

@author: Mariano del Río
@date: 20220704
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
