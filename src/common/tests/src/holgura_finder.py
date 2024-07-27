#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls, CarState
from std_msgs.msg import Int16, Bool
import math
import time


def send_command(delta):
    msg = Controls()
    msg.steering = delta
    control_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('holgura_finder')

    control_publisher = rospy.Publisher('/controls', Controls, queue_size=1)
    rate = rospy.Rate(10)

    ini = 0.4
    paso = 0.01
    i=0
    while not rospy.is_shutdown():
        rospy.loginfo("GIRANDO "+str(ini+i*paso)+" grados")
        for _ in range(50):
            send_command(ini+i*paso)
            rate.sleep()
            send_command(-ini-i*paso)
            rate.sleep()
        i+=1



