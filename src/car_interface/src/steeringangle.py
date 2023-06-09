import threading
import serial
import rospy


class SteeringAngle(threading.Thread):
    ZERO_OFFSET = 325
    MULTIPLICATOR = 2

    def __init__(self, serial_port):
        try:
            self.ser = serial.Serial(serial_port, baudrate=9600)
            rospy.loginfo('Succesfully started steering angle serial')
            self.wheel_angle = 0
            self._stop_event = threading.Event()
        except serial.PortNotOpenError:
            rospy.logerr(f'Could not open serial port on {serial_port}')


    def run(self):
        while not self._stop_event:
            self.ser.write([0xFF])
            angle = (int(self.ser.readline()) - ZERO_OFFSET)*MULTIPLICATOR
            # Redundant, but more thread safe
            self.wheel_angle = angle

    def get_wheel_angle(self):
        return self.wheel_angle

    def stop(self):
        self._stop_event.set()
        self.ser.close()
