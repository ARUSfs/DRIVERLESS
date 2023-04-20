#!/usr/bin/env python3

import rospy
from slam_handle import *

def main():
    rospy.init_node("map", anonymous=True)
    slam = FastSLAM2()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
