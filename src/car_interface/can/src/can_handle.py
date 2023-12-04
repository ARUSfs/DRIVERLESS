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
from common_msgs.msg import Controls

class CANHandle:
    """ Class to manage CAN communication """
    def __init__(self):
        self.MAX_ACC = 0.2
        """Initialize the CAN Handle by subscribing and publishing topics and starting the main asynchronous loop."""
        #   DESCOMENTAR CON CUIDADO, ENVIARÁ COMANDAS AL INVERSOR
        ###   self.subscribe_topics()
        #   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.publish_topics()
        """Start the main asynchronous loop."""
        loop = asyncio.get_event_loop()
        loop.run_until_complete(asyncio.gather(
            self.get_and_send_motor_speed(),
            ))
    def subscribe_topics(self):
        rospy.Subscriber("/controls",Controls, self.cmd_callback)

    def publish_topics(self):
        self.speed_publisher = rospy.Publisher("/motor_speed", Float32, queue_size=10)

    def cmd_callback(self, msg: Controls):
        acc =  msg.accelerator
        acc = min(acc,self.MAX_ACC)
        datos_comanda = list(int.to_bytes(int(acc*(2**15))-1, byteorder='little', length=2, signed=True))
        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x90]+datos_comanda)
        print(msg)
        with can.Bus(interface='socketcan', channel='can0', bitrate=500000) as bus:
            bus.send(msg)

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

                    #rospy.logerr("Velocity: " + str(final_v))
                    self.speed_publisher.publish(final_v)
