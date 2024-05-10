#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls
from std_msgs.msg import Float32, Int16
import math
import time

DURATION = 25000
AMPLITUDE = 15.0
FREQUENCY = 0.2
TARGET_SPEED = 1

AS_status = 0

kp=1
ki=0
kd=0

Cp=0
Ci=0
Cd=0

current_time=0
start_time=0
prev_time = 0
prev_error = 0


def AS_status_callback(AS_status_msg: Int16):
        global start_time, AS_status
        if AS_status != 0x02  and AS_status_msg.data==0x02:
            start_time = rospy.get_time()
        AS_status = AS_status_msg.data


def speed_callback(motor_speed_msg: Float32):
        global current_time, start_time, AS_status
        current_time = time.time()
        control_msg = Controls()


        # Create a Controls message and assign sinusoidal steering and calculated acceleration
        # Use PID controller to get the accelerator value
        if(motor_speed_msg.data>0.75):
            control_msg.steering = generate_sinusoidal_steering(current_time-start_time)
        else:
            control_msg.steering = 0

   
        control_msg.accelerator = accelerator_control(motor_speed_msg.data, TARGET_SPEED)
        #control_msg.accelerator = 0

        # Publish the message
        #rospy.loginfo(AS_status)
        if current_time-start_time<DURATION and AS_status==0x02:
            control_publisher.publish(control_msg)
            rospy.loginfo(control_msg)

def generate_sinusoidal_steering(time):
    steering_angle = AMPLITUDE * math.cos(2 * math.pi * FREQUENCY * time)
    return steering_angle

def accelerator_control(current: float, target: float):
        global current_time, prev_time, prev_error, Cp, Ci, Cd

        error = target - current


        dt = current_time-prev_time

        # Calculate gains
        de = error-prev_error
        Cp = error
        Ci += error*dt
        Cd = 0

        if dt > 0:
            Cd = de/dt

        # Update time and error
        prev_time = current_time
        prev_error = error

        cmd = kp*Cp + ki*Ci + kd*Cd

        return min(cmd, 0.2) 

if __name__ == '__main__':
    rospy.init_node('sinusoidal_control_node')

    start_time = time.time()

    rospy.Subscriber('/can/AS_status', Int16, AS_status_callback,queue_size=1)

    control_publisher = rospy.Publisher('/controls', Controls, queue_size=1)
    rospy.Subscriber('/motor_speed', Float32, speed_callback,queue_size=1)

    rospy.spin()





