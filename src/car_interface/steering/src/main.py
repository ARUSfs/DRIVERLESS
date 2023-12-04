#!/usr/bin/env python3
"""Script to initialize EPOS handler

@author: Jacobo Pindado
@date: 06/06/2023
"""
import rospy

from steering_handle import SteeringHandle


if __name__ == '__main__':
    rospy.init_node('car_interface', anonymous=False)
    try:
        steering_h = SteeringHandle()
        rospy.on_shutdown(steering_h.clean_and_close)
        rospy.spin()
    except:
        steering_h.clean_and_close()
