#!/usr/bin/env python3
""" Script to manage CAN communication.

@author: Jorge M.
@date: 18/11/2023
"""
import can
import math
import rospy
import asyncio
from std_msgs.msg import Float32

class CANHandle:
    """ Class to manage CAN communication """
    def __init__(self):
        """Initialize the CAN Handle by subscribing and publishing topics and starting the main asynchronous loop."""
        self.subscribe_topics()
        self.publish_topics()
        """Start the main asynchronous loop."""
        loop = asyncio.get_event_loop()
        loop.run_until_complete(asyncio.gather(
            #self.get_and_send_motor_speed(),
            self.async_function(),
            self.other_async_function()
            ))
    def subscribe_topics(self):
        pass
    
    def publish_topics(self):
        self.speed_publisher = rospy.Publisher("/motor_speed", Float32, queue_size=10)

    async def get_and_send_motor_speed(self):
        """Retrieve motor speed and send it over ROS."""
        vel_max = 5500.0
        wheel_radius = 0.2
        transmission_ratio = 11/45

        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x3D, 0x30, 0x0A])

        with can.Bus(interface='socketcan', channel='can0', bitrate=500000) as bus:
            bus.send(msg)
            for msg in bus:
                if msg.arbitration_id == 0x181 and int(msg.data[0]) == 0x30:
                    int_val = int.from_bytes(msg.data[1:3], byteorder='little', signed=True)
                    angular_v = int_val / 2**15 * vel_max
                    final_v = -angular_v * 2*math.pi*wheel_radius*transmission_ratio/60

                    print("Velocity: " + str(final_v))
                    self.speed_publisher.publish(final_v)

    async def async_function(self):
        x = 0
        while True:
            rospy.logerr("Hello: " + str(x))
            x += 1
            await asyncio.sleep(1)

    async def other_async_function(self):
        x = 0
        while True:
            rospy.logerr("Bye: " + str(x))
            x += 1
            await asyncio.sleep(1)
    