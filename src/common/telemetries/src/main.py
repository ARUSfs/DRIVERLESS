#!/usr/bin/env python3
import rospy
from writter import DataLoggerNode

def main():
    rospy.init_node('DataLoggerNode', anonymous=True)
    DataLoggerNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass