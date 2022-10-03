#!/usr/bin/env python

'''
Script to initialize fssim interface node

@author: Mariano del Río
@date: 20220524
'''
import rospy
from transformer import transformerFssim


def main():

    rospy.init_node('fssim_interface', anonymous=True)
    transformer_fssim = transformerFssim()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
