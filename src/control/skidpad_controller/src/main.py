#!/usr/bin/env python3
"""Script to initialize skidpad controller node

@author: Javier Utrilla
@date: 20240531
"""
import rospy
from SkidpadController import SkidpadController


def main():

    rospy.init_node('skidpad_controller', anonymous=True)
    SkidpadController()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass