#!/usr/bin/env python3

import rospy
from slam_handle import *

def main():
    rospy.init_node("slam_map", anonymous=False)
    slam = FastSLAM2()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        slam.publish_map()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
