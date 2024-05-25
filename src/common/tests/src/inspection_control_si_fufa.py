#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls
from std_msgs.msg import Float32, Int16, Bool
import math
import time


PUSHING = False
DURATION = 1000 #s
TARGET_DURATION = 20 #s

AMPLITUDE = 20 #degrees
FREQUENCY = 0.1 #s
KP=0.5

TARGET_SPEED = 2 #m/s
MAX_CMD = 0.2 
MIN_CMD = 0


def AS_status_callback(AS_status_msg: Int16):
        global AS_status, start_time
        AS_status = AS_status_msg.data
        if AS_status == 2 and start_time==0:
              start_time = time.time()


def speed_callback(motor_speed_msg: Float32):
        global start_time,braking,actual_speed,last_speed_update

        if AS_status==0x02 and not braking:

            control_msg = Controls()
            control_msg.steering = generate_sinusoidal_steering(time.time()-start_time)
            if PUSHING:
                control_msg.accelerator = 0
            else:
                control_msg.accelerator = accelerator_control(motor_speed_msg.data, TARGET_SPEED)

            control_publisher.publish(control_msg)
            rospy.logerr(control_msg)

def generate_sinusoidal_steering(time):
    steering_angle = AMPLITUDE * math.sin(2*math.pi*FREQUENCY *time)
#     steering_angle = AMPLITUDE * math.sin(FREQUENCY * time*time)
    return steering_angle

def accelerator_control(current: float, target: float):
        global start_time,target_reached_time,braking

        error = target - current

        cmd = KP*error

        if abs(error)<target*0.01 and target_reached_time==0:
            target_reached_time = time.time()

        if (target_reached_time!=0 and time.time()-target_reached_time>TARGET_DURATION) or (time.time()-start_time > DURATION):
            braking = True
            braking_msg = Bool()
            braking_msg.data = True
            braking_publisher.publish(braking_msg)

        return max(min(cmd, MAX_CMD),MIN_CMD) 

if __name__ == '__main__':
    rospy.init_node('sinusoidal_control_node')

    start_time = 0
    target_reached_time = 0
    AS_status = 0

    braking = False

    control_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1)
    braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
    rospy.Subscriber('/motor_speed', Float32, speed_callback,queue_size=1)
    rospy.Subscriber('/can/AS_status', Int16, AS_status_callback,queue_size=1)

    rospy.spin()