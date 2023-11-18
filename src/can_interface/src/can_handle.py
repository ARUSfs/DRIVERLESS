#!/usr/bin/env python3
""" Script to manage CAN communication.

@author: Jorge M.
@date: 18/11/2023
"""
import can
import math
import rospy
from std_msgs.msg import Float32

class CANHandle:
    """ Class to manage CAN communication """
    def __init__(self):
        self.subscribe_topics()
        self.publish_topics()

    def subscribe_topics(self):
        """ Subscribe to CAN topics """
        pass
    
    def publish_topics(self):
        """ Publish to CAN topics """
        self.speed_publisher = rospy.Publisher("/motor_speed", Float32, queue_size=10)

    def getAndSendMotorSpeed(self):
        """ Function to get motor speed and send it to ROS """
        vel_max = 5500.0
        wheel_radius = 0.2
        transmission_ratio = 11/45

        """ Message to request motor speed every 10ms"""
        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x3D, 0x30, 0x0A])

        """ Send message and wait for response"""
        with can.Bus(interface='socketcan', channel='can0', bitrate=500000) as bus:
            bus.send(msg)
            for msg in bus:
                if msg.arbitration_id == 0x181 and int(msg.data[0]) == 0x30:
                    int_val = int.from_bytes(msg.data[1:3], byteorder='little', signed=True)
                    angular_v = int_val / 2**15 * vel_max
                    final_v = -angular_v * 2*math.pi*wheel_radius*transmission_ratio/60
                    print("Velocidad: " + final_v)

                    """ Publish speed to ROS """
                    self.speed_publisher.publish(final_v)

                    
    