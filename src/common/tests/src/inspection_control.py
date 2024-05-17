#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls
from std_msgs.msg import Float32, Int16
import math
import time


AMPLITUDE = 20
FREQUENCY = 0.2
TARGET_SPEED = 0.5

AS_status = 0

kp=0.5

current_time=0
prev_time = 0
prev_error = 0


def AS_status_callback(AS_status_msg: Int16):
        global AS_status
        AS_status = AS_status_msg.data


def speed_callback(motor_speed_msg: Float32):
        current_time = rospy.get_time()

        control_msg = Controls()
        control_msg.steering = generate_sinusoidal_steering(current_time)
        # control_msg.accelerator = accelerator_control(motor_speed_msg.data, TARGET_SPEED)
        control_msg.accelerator = 0

        if AS_status==0x02:
            control_publisher.publish(control_msg)

def generate_sinusoidal_steering(time):
    steering_angle = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * time)
    return steering_angle

def accelerator_control(current: float, target: float):

        error = target - current

        cmd = kp*error

        return min(cmd, 0.2) 

if __name__ == '__main__':
    rospy.init_node('sinusoidal_control_node')

    start_time = time.time()

    control_publisher = rospy.Publisher('/controls', Controls, queue_size=1)
    rospy.Subscriber('/motor_speed', Float32, speed_callback,queue_size=1)
    rospy.Subscriber('/can/AS_status', Int16, AS_status_callback,queue_size=1)

    rospy.spin()