#!/usr/bin/env python3
"""Script to initialize interface node

@author: Mariano del Río
@date: 20221019
"""
import rospy
from controller import Controller


def main():

    rospy.init_node('controller', anonymous=True)
    Controller()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
