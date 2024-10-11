#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls, CarState
from std_msgs.msg import Int16, Bool, Float32
import math
import time

KP = 43.87
KI = 1.29

profile = []

def as_status_callback(msg:Int16):
    global AS_status, start_time
    AS_status = msg.data
    if (AS_status == 2 and start_time == 0):
        start_time=time.time()
        last_time = time.time()

def speed_callback(msg: Float32):
        global start_time,braking

        if not braking:
            
            v = msg.data
            control_msg = Controls()

            control_msg.steering = 0

            
            control_msg.accelerator = feed_forward(v)

            control_publisher.publish(control_msg)
            #rospy.logerr(control_msg)


def feed_forward(current: float):
        global start_time,braking,last_time, integral

        target = 2
        prev_target = 2

        t = time.time()-start_time
        dt = time.time()-last_time
        last_time = time.time()

        error = target - current
        integral += error*dt
        cmd_pid = KP*error + KI*integral

        target_acc = (target-prev_target)/dt
        target_acc = target_acc*240*0.2
        cmd = cmd_pid + target_acc
        
        return max(min(cmd/230, 1),-1)

if __name__ == '__main__':
    rospy.init_node('sinusoidal_control_node')

    start_time = 0
    AS_status = 0
    last_time = 0
    braking = False
    integral = 0

    control_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1)
    braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
    rospy.Subscriber('/wheel_speed', Float32, speed_callback,queue_size=1)
    rospy.Subscriber('/can/AS_status',Int16, as_status_callback, queue_size=1)
    rospy.spin()
