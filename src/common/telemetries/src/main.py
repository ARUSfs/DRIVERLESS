#!/usr/bin/env python3
import rospy
from writter import DataLoggerNode

sim_mode = rospy.get_param('/control_pure_pursuit/simulation')

def main():
    rospy.init_node('DataLoggerNode', anonymous=True)
    if sim_mode:
        DataLoggerNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass