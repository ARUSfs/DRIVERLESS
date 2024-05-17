#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls
from std_msgs.msg import Float32, Int16
import math
import time

AMPLITUDE = 19
FREQUENCY = 0.1
#0.2, 0.3, 0.5, 0.1, 0.3
TARGET_SPEED = [3,5]
STABLE_DURATION = 0.5
#3.5, 4, 4, 1, 2
BRAKE_DURATION = 5
BRAKE_THRESHOLD = 2
SPEED_MARGIN = 0.05 # 5%



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
brake_start_time = 0
target_reached_time=0
braking = False

i=0


def AS_status_callback(AS_status_msg: Int16):
        global start_time, AS_status
        AS_status = AS_status_msg.data
        if AS_status == 0x02:
              start_time = rospy.get_time()


def speed_callback(motor_speed_msg: Float32):
        global current_time, start_time, AS_status, braking, brake_start_time, i, target_reached_time
        current_time = time.time()

        # Create a Controls message and assign sinusoidal steering and calculated acceleration
        # Use PID controller to get the accelerator value
        control_msg = Controls()
        if motor_speed_msg.data < 0.5:
            control_msg.steering = 0
        else:
            control_msg.steering = generate_sinusoidal_steering(current_time-start_time)
        # control_msg.steering = 0

        if braking:
            actual_target = brake_control(TARGET_SPEED[-1],motor_speed_msg.data)
        else:
            actual_target = TARGET_SPEED[i]

        if check_target_reached(actual_target,motor_speed_msg.data) and not braking:
            if (i<len(TARGET_SPEED)-1):
                i+=1
                target_reached_time=0
            else:
                braking = True
                brake_start_time=time.time()


        
        control_msg.accelerator = accelerator_control(motor_speed_msg.data, actual_target)

        if AS_status==0x02:
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

        return max(min(cmd, 1),-1) 
    
def brake_control(target_speed: float,vel: float):
    global brake_start_time
    
    brake_target = ((BRAKE_THRESHOLD - target_speed) * (time.time() - brake_start_time))/BRAKE_DURATION + target_speed
    brake_target = max(brake_target,BRAKE_THRESHOLD)

    if vel < BRAKE_THRESHOLD*(1+SPEED_MARGIN):
        brake_target = 0
        msg = Int16()
        msg.data = 3
        AS_publisher.publish(msg)
        
        # rospy.loginfo("Braking completed, speed below threshold")
        
    return brake_target

def check_target_reached(actual_target,vel):
    global target_reached_time
    if actual_target*(1 - SPEED_MARGIN) < vel < actual_target*(1 + SPEED_MARGIN):
        if target_reached_time==0:
            target_reached_time = time.time()
        elif time.time()-target_reached_time > STABLE_DURATION:
            return True
    else:
        return False

    

         



if __name__ == '__main__':
    rospy.init_node('sinusoidal_control_node')

    start_time = time.time()

    rospy.Subscriber('/can/AS_status', Int16, AS_status_callback,queue_size=1)

    control_publisher = rospy.Publisher('/controls', Controls, queue_size=1)
    AS_publisher = rospy.Publisher('/can/AS_status', Int16, queue_size=1)
    rospy.Subscriber('/motor_speed', Float32, speed_callback,queue_size=1)

    rospy.spin()
    
