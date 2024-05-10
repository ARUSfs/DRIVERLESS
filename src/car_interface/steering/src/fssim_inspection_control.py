#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls
from fssim_common.msg import State, Cmd
from std_msgs.msg import Float32, Int16
import math
import time

DURATION = 25000
AMPLITUDE = 19.0
FREQUENCY = 0.1
TARGET_SPEED = 2

AS_status = 0

kp=0.5
ki=0
kd=0

Cp=0
Ci=0
Cd=0

current_time=0
start_time=0
prev_time = 0
prev_error = 0



def speed_callback(state_msg: State):
        global current_time, start_time
        current_time = time.time()

        if state_msg.vx < 0.2:
            start_time = time.time()
        # rospy.loginfo(current_time-start_time)

        control_msg = Cmd()
        control_msg.delta = generate_sinusoidal_steering(current_time-start_time)*math.pi/180
        control_msg.dc = accelerator_control(math.hypot(state_msg.vx,state_msg.vy), TARGET_SPEED)
       
        if current_time-start_time<DURATION:
                control_publisher.publish(control_msg)
                rospy.loginfo(control_msg)

def generate_sinusoidal_steering(time):
    steering_angle = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * time)
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

    control_publisher = rospy.Publisher('/fssim/cmd', Cmd, queue_size=1)
    rospy.Subscriber('/fssim/base_pose_ground_truth', State, speed_callback,queue_size=1)

    rospy.spin()





