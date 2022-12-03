#!/usr/bin/env python3
"""Script to initialize tree tf

@author: Mariano del Río
@date: 20221104
"""

import rospy
from car_broadcaster_handle import CarBroadcasterHandle


def main():

    rospy.init_node('car_broadcaster', anonymous=True)
    CarBroadcasterHandle()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
