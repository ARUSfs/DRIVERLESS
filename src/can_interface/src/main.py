#!/usr/bin/env python3
""" Script to initialize CAN interface handler

@author: Jorge M.
@date: 18/11/2023
"""

import rospy
from can_handle import CANHandle

def main():
    rospy.init_node('can_interface', anonymous=True)
    can_handle = CANHandle()
    rospy.spin()
    


