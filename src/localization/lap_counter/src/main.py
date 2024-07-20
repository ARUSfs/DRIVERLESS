#!/usr/bin/env python3

import rospy
from lap_counter import Lap_counter

def main():
    rospy.init_node('lap_counter', anonymous=True)
    lap_counter = Lap_counter()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass