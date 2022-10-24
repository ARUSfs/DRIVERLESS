#!/usr/bin/env python
"""Script to initialize planning node

@author: Mariano del Río
@date: 20220504
"""

import rospy
from planning_handle import PlanningHandle


def main():

    rospy.init_node('delaunay_detector', anonymous=True)
    PlanningHandle()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
