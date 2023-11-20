#!/usr/bin/env python3
""" Script to initialize CAN interface handler

@author: Jorge M.
@date: 18/11/2023
"""

import rospy
from can_handle import CANHandle

def main():
    """Initialize and run the CAN interface node."""
    rospy.init_node('can_interface', anonymous=True)
    CANHandle()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


