#!/usr/bin/env python3
"""Script to initialize tree tf

@author: Jorge Muñoz    
@date: 20221125
"""

import rospy
from handle import Localization

def main():
    rospy.init_node('car_broadcaster', anonymous=True)
    Localization()
    rospy.spin()
    rospy.logwarn("spin")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass