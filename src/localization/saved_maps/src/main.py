#!/usr/bin/env python3

import rospy
from saved_map import *

def main():
    rospy.init_node('saved_map', anonymous=True)
    saved_map = SavedMap()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass