#!/usr/bin/env python3

'''
Script to control the car with a Dualshock4

@author: Rafael Guil Valero
@date: 15/05/24
'''
import rospy
from joystick_handle import joystick


def main():

    rospy.init_node('joystick', anonymous=True)
    joystick()  #Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
