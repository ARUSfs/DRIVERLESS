#!/usr/bin/env python3
"""Script to initialize the visual perception node

@author: Jacobo Pindado
@date 20221113
"""

import rospy
from estimator_handle import EstimatorHandle


def main():
    rospy.init_node("vision_cone_detector", anonymous=True)
    EstimatorHandle()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
