#!/usr/bin/env python
"""Script to initialize interface node

@author: Mariano del Río
@date: 20221019
"""
import rospy
from interface_handle import InterfaceHandle


def main():

    rospy.init_node('interface', anonymous=True)
    InterfaceHandle()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
