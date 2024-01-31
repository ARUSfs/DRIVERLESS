#!/usr/bin/env python3

import rospy
from data_association_icp import *

def main():
    rospy.init_node('greedy_data_association', anonymous=True)
    data_association = Data_association2()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass