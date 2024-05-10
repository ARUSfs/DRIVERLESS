#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray


def epos_info_callback(msg: Float32MultiArray):
    pMovementState = int(msg.data[0]).to_bytes(1, byteorder='little')
    pPosition = int(msg.data[1]*100).to_bytes(3, byteorder='little', signed=True)
    pTargetPosition = int(msg.data[2]*100).to_bytes(3, byteorder='little', signed=True)
    pVelocity = int(msg.data[3]*100).to_bytes(2, byteorder='little', signed=True)
    pVelocityAvg = int(msg.data[4]*100).to_bytes(2, byteorder='little', signed=True)
    pTorque = int(msg.data[6]).to_bytes(2, byteorder='little', signed=True)

    rospy.loginfo(pMovementState)
    rospy.loginfo(pPosition)
    rospy.loginfo(pTargetPosition)
    rospy.loginfo(pVelocity)
    rospy.loginfo(pVelocityAvg)
    rospy.loginfo(pTorque)

if __name__ == '__main__':
    rospy.init_node('car_interface', anonymous=False)
    rospy.Subscriber('/steering/epos_info', Float32MultiArray, epos_info_callback)
    rospy.spin()


