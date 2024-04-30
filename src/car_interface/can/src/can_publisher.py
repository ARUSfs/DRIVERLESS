import rospy
import can
from can_reader import CanReader
from common_msgs.msg import Controls
import threading

class CanPublisher:

    def __init__(self):

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        #--------------------------- PELIGRO COMANDAS DE PAR AL INVERSOR!!! --------------------------------------------------
        #rospy.Subscriber("/controls",Controls, self.cmd_callback)
        #---------------------------------------------------------------------------------------------------------------------
        
        rospy.Timer(rospy.Duration(0.5), self.publish_temp)

        self.temp = 0
        self.MAX_ACC = 0.2
        self.motor_speed_req()
        self.canReader = CanReader()
        self.can_reader_thread = threading.Thread(target=self.canReader.read)
        self.can_reader_thread.daemon = True  # Hacer que el hilo se detenga cuando el programa principal se detenga
        self.can_reader_thread.start()
        self.canReader.read()
        
    
    def cmd_callback(self, msg: Controls):
        acc =  msg.accelerator
        acc = min(acc,self.MAX_ACC)
        datos_comanda = list(int.to_bytes(int(acc*(2**15))-1, byteorder='little', length=2, signed=True))
        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x90]+datos_comanda)
        self.bus.send(msg)

    
    def publish_temp(self, event):

        temp_file = "/sys/class/thermal/thermal_zone6/temp"
        try:
            with open(temp_file, "r") as f:
                t = f.readline()
                self.temp = int(t) / 1000
                
        except Exception as e:
            print("Error: ", e)
        
        bytes_temp = int(self.temp*10).to_bytes(2, byteorder='little')
        msg = can.Message(arbitration_id=0x183, is_extended_id=False, data=[0x01]+list(bytes_temp))
        rospy.loginfo(msg)
        self.bus.send(msg)

    
    def motor_speed_req(self):
        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x3D, 0x30, 0x0A])
        self.bus.send(msg)
        
