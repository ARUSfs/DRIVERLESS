#!/usr/bin/env python3

'''
Script to initialize fssim interface node

@author: Mariano del Río
@date: 20220524
'''
import rospy
from transformer import transformerFssim


def main():

    rospy.init_node('fssim_interface', anonymous=True)
    transformerFssim()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
