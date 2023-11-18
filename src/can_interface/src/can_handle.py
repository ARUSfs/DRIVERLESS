#!/usr/bin/env python3
""" Script to manage CAN communication.

@author: Jorge M.
@date: 18/11/2023
"""
import can

class CANHandle:
    """ Class to manage CAN communication """
    def __init__(self, channel='can0', bustype='socketcan'):
        """ Constructor """
        self.channel = channel
        self.bustype = bustype
        self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)

    def send(self, msg):
        """ Send CAN message """
        self.bus.send(msg)

    def receive(self):
        """ Receive CAN message """
        return self.bus.recv()

    def close(self):
        """ Close CAN interface """
        self.bus.shutdown()