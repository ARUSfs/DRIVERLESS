#!/usr/bin/env python3
"""Script to initialize tree tf

@author: Jorge Muñoz    
@date: 20221125
"""

import rospy
from handle import localization

def main():

    rospy.init_node('car_broadcaster', anonymous=True)
    localization()
    rospy.spin()


if __name__ == 'main':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
