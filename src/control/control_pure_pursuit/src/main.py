#!/usr/bin/env python
"""
Script to initialize control node

@author: Mariano del Río
@date: 20220704
"""

import rospy
from control_handle import controlHandle


def main():

    rospy.init_node('control_pure_pursuit', anonymous=True)
    frequency = rospy.get_param('/control_pure_pursuit/frequency')
    rate = rospy.Rate(frequency)

    control_handle = controlHandle()
    while not rospy.is_shutdown():

        control_handle.run()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
