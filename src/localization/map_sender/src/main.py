#!/usr/bin/env python3

import rospy
from map_sender import *

def main():
    rospy.init_node('map_sender', anonymous=True)
    map_sender = Map_sender()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass