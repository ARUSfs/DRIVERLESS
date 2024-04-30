#!/usr/bin/env python3
import rospy
from can_publisher import CanPublisher

def main():
    """Initialize and run the CAN interface node."""
    rospy.init_node('can_reader', anonymous=True)
    CanPublisher()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass