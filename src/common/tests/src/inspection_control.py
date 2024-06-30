#!/usr/bin/env python3
import rospy
from common_msgs.msg import Controls, CarState
from std_msgs.msg import Int16, Bool
import math
import time


PUSHING = False
DURATION = 1000 #s
TARGET_DURATION = 1000 #s

AMPLITUDE = 20 #degrees
FREQUENCY = 0.2 #s
KP=0.5

TARGET_SPEED = 3 #m/s


def speed_callback(msg: CarState):
        global start_time,braking,actual_speed,last_speed_update

        if not braking:
            
            v = math.hypot(msg.vx,msg.vy)
            control_msg = Controls()

            control_msg.steering = generate_sinusoidal_steering(time.time()-start_time)

            if PUSHING:
                control_msg.accelerator = 0
            else:
                control_msg.accelerator = accelerator_control(v, TARGET_SPEED)

            control_publisher.publish(control_msg)
            #rospy.logerr(control_msg)

def generate_sinusoidal_steering(time):
    steering_angle = AMPLITUDE * math.sin(2*math.pi*FREQUENCY *time)
#     steering_angle = AMPLITUDE * math.sin(FREQUENCY * time*time)
    return steering_angle

def accelerator_control(current: float, target: float):
        global start_time,target_reached_time,braking

        error = target - current
        cmd = KP*error

        #if abs(error)<target*0.01 and target_reached_time==0:
        #    target_reached_time = time.time()

        #if (target_reached_time!=0 and time.time()-target_reached_time>TARGET_DURATION) or (time.time()-start_time > DURATION):
        #   braking = True
        #   braking_msg = Bool()
        #   braking_msg.data = True
        #   braking_publisher.publish(braking_msg)
        #   rospy.logerr("Frenando")

        return max(min(cmd, 1),-1)

if __name__ == '__main__':
    rospy.init_node('sinusoidal_control_node')

    start_time = 0
    target_reached_time = 0
    AS_status = 0


    braking = False

    control_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1)
    braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
    rospy.Subscriber('/car_state/state', CarState, speed_callback,queue_size=1)
    #rospy.Subscriber('/can/AS_status', Int16, AS_status_callback,queue_size=1)

    rospy.spin()
