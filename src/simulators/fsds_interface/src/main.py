#!/usr/bin/env python3

'''
Script to initialize fssim interface node

@author: Rafael Guil Valero
@date: 20220524
'''
import rospy
from transformer import transformerFSDS


def main():

    rospy.init_node('fsds_interface', anonymous=True)
    transformerFSDS()  # Run node
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
