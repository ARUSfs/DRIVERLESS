import rospy
import can
from can_reader import CanReader
from std_msgs.msg import Float32MultiArray, Int16
from common_msgs.msg import Controls
from ctypes import *
import threading

class CanPublisher:

    def __init__(self):
        
        self.bus0 = can.interface.Bus(channel='can0', bustype='socketcan')
        self.bus1 = can.interface.Bus(channel='can1', bustype='socketcan')

        #--------------------------- PELIGRO COMANDAS DE PAR AL INVERSOR!!! --------------------------------------------------
        rospy.Subscriber("/controls",Controls, self.cmd_callback)
        #---------------------------------------------------------------------------------------------------------------------
        rospy.Subscriber("/steering/epos_info", Float32MultiArray, self.epos_info_callback)
        rospy.Subscriber("/can/AS_status", Int16, self.pub_as_status)

        rospy.Timer(rospy.Duration(0.5), self.publish_temp)
        rospy.Timer(rospy.Duration(0.1), self.heart_beat)


        self.temp = 0
        self.MAX_ACC = 0.1
        self.motor_speed_req()
        self.canReader = CanReader()
        self.can_reader_thread0 = threading.Thread(target=self.canReader.read_can0)
        self.can_reader_thread1 = threading.Thread(target=self.canReader.read_can1)
        self.can_reader_thread0.daemon = True  # Hacer que el hilo se detenga cuando el programa principal se detenga
        self.can_reader_thread1.daemon = True
        self.can_reader_thread0.start()
        self.can_reader_thread1.start()


    def cmd_callback(self, msg: Controls):
        #rospy.loginfo("Peligro")
        acc =  msg.accelerator
        acc = min(acc,self.MAX_ACC)
        datos_comanda = list(int.to_bytes(int(acc*(2**15))-1, byteorder='little', length=2, signed=True))
        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x90]+datos_comanda)
        self.bus0.send(msg)

    def pub_as_status(self, msg: Int16):
        if msg.data==3:
            rospy.logwarn("AS FINISHED")
            m = can.Message(arbitration_id=0x202, is_extended_id=False, data=[0x01,0x01,0x03])
            self.bus0.send(m)

    
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
        #rospy.loginfo(msg)
        self.bus1.send(msg)

    
    def motor_speed_req(self):
        msg = can.Message(arbitration_id=0x201, is_extended_id=False, data=[0x3D, 0x30, 0x0A])
        self.bus0.send(msg)

    def heart_beat(self, event):
        msg = can.Message(arbitration_id=0x183, is_extended_id=False, data=[0x00])
        self.bus0.send(msg)


    def epos_info_callback(self, msg: Float32MultiArray):
        
        pMovementState = int(msg.data[0])
        pPosition = int(msg.data[1]*100).to_bytes(3, byteorder='little', signed=True)
        pTargetPosition = int(msg.data[2]*100).to_bytes(3, byteorder='little', signed=True)
        pVelocity = int(msg.data[3]*100).to_bytes(3, byteorder='little', signed=True)
        pVelocityAvg = int(msg.data[4]*100).to_bytes(3, byteorder='little', signed=True)
        pTorque = int(msg.data[5]).to_bytes(2, byteorder='little', signed=True)

        msgEposState = can.Message(arbitration_id=0x183, is_extended_id=False, data=[0x02, pMovementState, pPosition[0], pPosition[1], pPosition[2], pTargetPosition[0], pTargetPosition[1], pTargetPosition[2]])
        msgEposVelocity = can.Message(arbitration_id=0x183, is_extended_id=False, data=[0x03, pVelocity[0], pVelocity[1],pVelocity[2], pVelocityAvg[0], pVelocityAvg[1], pVelocityAvg[2]])
        msgEposTorque = can.Message(arbitration_id=0x183, is_extended_id=False, data=[0x04, pTorque[0], pTorque[1]])

        self.bus1.send(msgEposState)
        self.bus1.send(msgEposVelocity)
        self.bus1.send(msgEposTorque)



        
        
