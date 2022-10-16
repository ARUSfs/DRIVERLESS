#!/usr/bin/env python
"""
Script to initialize planning node

@author: Mariano del Río
@date: 20220504
"""

import rospy
from planning_handle import planningHandle


def main():

    rospy.init_node('delaunay_detector', anonymous=True)
    planningHandle()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
