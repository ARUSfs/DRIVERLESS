#!/usr/bin/env python3
import rospy
from state_handle import StateClass

def main():
    rospy.init_node('state_node', anonymous=True)
    state_class = StateClass()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass