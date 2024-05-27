import rospy
import can
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Int16
import numpy as np
from geometry_msgs.msg import Vector3

vel_max = 5500.0
wheel_radius = 0.2
transmission_ratio = 11/45

class CanReader:
    def __init__(self):

        self.IMU_msg = Imu()
        self.GPS_pos = Vector3()
        self.as_status = 0
 
        #--------------------------------------------------------
        self.steering_extensometer = rospy.Publisher('/can/extensometer', Float32, queue_size=10)
        self.AS_status_pub = rospy.Publisher('/can/AS_status', Int16, queue_size=10)
        self.IMU_pub = rospy.Publisher('/IMU', Imu, queue_size=10)
        self.pGPS_loc = rospy.Publisher('GPS_location', NavSatFix, queue_size=10)
        self.pGPS_speed = rospy.Publisher('GPS_speed', Vector3, queue_size=10)
        self.invSpeed_pub = rospy.Publisher('/motor_speed', Float32, queue_size=10)
        
        self.bus0 = can.interface.Bus(channel='can0', bustype='socketcan')
        self.bus1 = can.interface.Bus(channel='can1', bustype='socketcan')
        self.bus0.set_filters([{"can_id": 0x181, "can_mask": 0x7FF, "extended": False}])
        self.bus1.set_filters([{"can_id": 0x182, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x380, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x394, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x392, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x384, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x382, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x185, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x205, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x187, "can_mask": 0x7FF, "extended": False},
                               {"can_id": 0x334, "can_mask": 0x7FF, "extended": False}])
        #--------------------------------------------------------

        rospy.Timer(rospy.Duration(1/500), self.publish_IMU)

       

    def read_can0(self):
        while not rospy.is_shutdown():

            message = self.bus1.recv()

            if message.arbitration_id == 0x182:
                sub_id = int(message.data[0])
                if sub_id == 0x00:
                    self.parse_AS_HB(message)
                elif sub_id == 0x01:
                    #rospy.loginfo("AS")
                    self.parse_as_status(message)
                elif sub_id == 0x02:
                    self.parse_fault_code(message)
                elif sub_id == 0x03:
                    self.parse_apps(message)
                elif sub_id == 0x04:
                    self.parse_brake_pressure(message)
                elif sub_id == 0x05:
                    self.parse_pneumatic_pressure(message)
                elif sub_id == 0x06:
                    self.parse_valves_state(message)

            elif message.arbitration_id == 0x380:
               #IMU acc
                self.parse_acc(message)

            elif message.arbitration_id == 0x394:
                #GPS pos
                self.parseGPS(message)

            elif message.arbitration_id == 0x392:
                #Velocidades GPS
                self.parse_vel_GPS(message)

            elif message.arbitration_id == 0x384:
                #Ángulos de Euler
                self.parse_euler_angles(message)
            
            elif message.arbitration_id == 0x382:
                self.parse_angular_velocity(message)
            
            elif message.arbitration_id == 0x185:
                sub_id = int(message.data[0])
                if sub_id == 0x00:
                    self.parse_dashboard_HB(message)
                elif sub_id == 0x01:
                    self.parse_mission(message)
                
            elif message.arbitration_id == 0x205:
                sub_id = int(message.data[0])
                if sub_id == 0x04:
                    self.parse_buzzer(message)

            elif message.arbitration_id == 0x187:
                sub_id = int(message.data[0])
                if sub_id == 0x01:
                    self.parse_steering_angle(message)
                elif sub_id == 0x02:
                    self.parse_front_extensometer(message)
                elif sub_id == 0x03:
                    self.parse_rear_extensometer(message)
                elif sub_id == 0x04:
                    self.parse_front_wheel_speed(message)
                elif sub_id == 0x05:
                    self.parse_rear_wheel_speed(message)

            elif message.arbitration_id == 0x334:
                self.parse_steering_angle(message)
               
    def read_can1(self):
        while not rospy.is_shutdown():
            message = self.bus0.recv()
            if message.arbitration_id == 0x181 and int(message.data[0]) == 0x30:
                #Inv speed
                self.parse_inv_speed(message)
   
   
    #Parsers --------------------------------------------------------
    #################################################### ACQUISITION ########################################################
    def parse_steering_angle(self, message):
        self.steering_angle = int.from_bytes(message.data[0:2], byteorder='little', signed=True)
        msg = Float32()
        msg.data = self.steering_angle
        self.steering_extensometer.publish(msg)
        #rospy.loginfo("Steering Angle: %f", self.steering_angle)
    
    def parse_front_extensometer(self, message):
        self.front_extensometer = int.from_bytes(message.data[1:3], byteorder='little', signed=True)*(10**-2)
        #rospy.loginfo("Front Extensometer: %f", self.front_extensometer)

    def parse_rear_extensometer(self, message):
        self.rear_extensometer = int.from_bytes(message.data[1:3], byteorder='little', signed=True)*(10**-2)
        #rospy.loginfo("Rear Extensometer: %f", self.rear_extensometer)
    
    def parse_front_wheel_speed(self, message):
        self.front_wheel_speed = int.from_bytes(message.data[1:3], byteorder='little', signed=True)*(10**-2)
        #rospy.loginfo("Front Wheel Speed: %f", self.front_wheel_speed)
    
    def parse_rear_wheel_speed(self, message):
        self.rear_wheel_speed = int.from_bytes(message.data[1:3], byteorder='little', signed=True)*(10**-2)
        #rospy.loginfo("Rear Wheel Speed: %f", self.rear_wheel_speed)


    ###################################################### DASHBOARD ########################################################
    def parse_dashboard_HB(self, message):
        self.dashboard_HB = int.from_bytes(message.data[1], byteorder='little', signed=False)
        #rospy.loginfo("Dashboard HB: %d", self.dashboard_HB)

    def parse_mission(self, message):
        self.mission = int.from_bytes(message.data[1], byteorder='little', signed=False)
        #rospy.loginfo("Mission: %d", self.mission)


    ###################################################### AS ########################################################
    def parse_AS_HB(self, message):
        #self.AS_HB = int.from_bytes(message.data[1], byteorder='little', signed=False)
        #rospy.loginfo("AS HB: %d", self.AS_HB)
        pass

    def parse_as_status(self, message):

        if(self.as_status != 2 and message.data[2]==2):
            rospy.sleep(2)
            rospy.loginfo("AS Status")
        self.as_status = message.data[2]
        #rospy.loginfo("AS Status: %d", self.as_status)
        
        msg = Int16()
        msg.data = self.as_status
        self.AS_status_pub.publish(msg)

    
    def parse_fault_code(self, message):
        self.asms_status = int.from_bytes(message.data[1:3], byteorder='little', signed=False)
        #rospy.loginfo("ASMS Status: %d", self.asms_status)

    def parse_apps(self, message):
        self.apps = int.from_bytes(message.data[1:3], byteorder='little', signed=False)
        #rospy.loginfo("APPS: %d", self.apps)

    def parse_valves_state(self, message):
        self.ebs_status = int.from_bytes(message.data[1:3], byteorder='little', signed=False)
        #rospy.loginfo("EBS Status: %d", self.ebs_status)

    def parse_brake_pressure(self, message):
        self.brake_pressure = int.from_bytes(message.data[1:3], byteorder='little', signed=False)
        #rospy.loginfo("Brake Pressure: %d", self.brake_pressure)

    def parse_pneumatic_pressure(self, message):
        self.pneumatic_pressure = int.from_bytes(message.data[1:3], byteorder='little', signed=False)
        #rospy.loginfo("Pneumatic Pressure: %d", self.pneumatic_pressure)

  
    ##################################################### IMU ##################################################
    def parse_acc(self, message):
        acc_x = int.from_bytes(message.data[0:2], byteorder='little', signed=True)*(10**-2)
        acc_y = int.from_bytes(message.data[2:4], byteorder='little', signed=True)*(10**-2)
        acc_z = int.from_bytes(message.data[4:6], byteorder='little', signed=True)*(10**-2)

        #rospy.loginfo("X: %f, Y: %f, Z: %f", acc_x, acc_y, acc_z)

        self.IMU_msg.linear_acceleration.x = acc_x
        self.IMU_msg.linear_acceleration.y = acc_y
        self.IMU_msg.linear_acceleration.z = acc_z
        self.IMU_msg.header.stamp = rospy.Time.now()

    
    def parseGPS(self, message):
        self.latitude= int.from_bytes(message.data[0:4], byteorder='little', signed=True)*(10**-7)
        self.longitude = int.from_bytes(message.data[4:8], byteorder='little', signed=True)*(10**-7)
        
        #rospy.loginfo("Lat: %f, Long: %f", self.latitude, self.longitude)

        v = NavSatFix()
        v.latitude = self.latitude
        v.longitude = self.longitude
        v.header.stamp = rospy.Time.now()

        self.pGPS_loc.publish(v)

    
    def parse_vel_GPS(self, message):
        velN = float(int.from_bytes(message.data[0:2], byteorder='little', signed=True))*(10**-2)
        velE = float(int.from_bytes(message.data[2:4], byteorder='little', signed=True))*(10**-2)
        velD = float(int.from_bytes(message.data[4:6], byteorder='little', signed=True))*(10**-2)
        
        #rospy.loginfo("VelN: %d, VelE: %d, VelD: %d", self.velN, self.velE, self.velD)

        v = Vector3()
        v.x = velN
        v.y = velE
        v.z = velD

        self.pGPS_speed.publish(v)
    
    
    def parse_euler_angles(self, message):
        roll = int.from_bytes(message.data[0:2], byteorder='little', signed=True)*(10**-4)
        pitch = int.from_bytes(message.data[2:4], byteorder='little', signed=True)*(10**-4)
        yaw = int.from_bytes(message.data[4:6], byteorder='little', signed=True)*(10**-4)

        #rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        self.IMU_msg.orientation.x = qx
        self.IMU_msg.orientation.y = qy
        self.IMU_msg.orientation.z = qz
        self.IMU_msg.orientation.w = qw
        self.IMU_msg.header.stamp= rospy.Time.now()

   
    def parse_angular_velocity(self, message):
            self.angular_velocity_x = int.from_bytes(message.data[0:2], byteorder='little', signed=True)*(10**-3)
            self.angular_velocity_y = int.from_bytes(message.data[2:4], byteorder='little', signed=True)*(10**-3)
            self.angular_velocity_z = int.from_bytes(message.data[4:6], byteorder='little', signed=True)*(10**-3)

            # rospy.loginfo("X: %f, Y: %f, Z: %f", self.angular_velocity_x, self.angular_velocity_y, self.angular_velocity_z)

            self.IMU_msg.angular_velocity.x = self.angular_velocity_x
            self.IMU_msg.angular_velocity.y = self.angular_velocity_y
            self.IMU_msg.angular_velocity.z = self.angular_velocity_z
            self.IMU_msg.header.stamp = rospy.Time.now()


########################################################## INVERSOR ########################################################    
    def parse_inv_speed(self, message):
        int_val = int.from_bytes(message.data[1:3], byteorder='little', signed=True)
        angular_v = int_val / 2**15 * vel_max
        self.inv_speed = -angular_v * 2*np.pi*wheel_radius*transmission_ratio/60

        #rospy.loginfo("Inv Speed: %f", self.inv_speed)

        self.invSpeed_pub.publish(self.inv_speed)


########################################################### BUZZER ########################################################
    def parse_buzzer(self, message):
        pass

    #Publishers ------------------------------------------------------------------------------------------

    def publish_IMU(self, event):
        self.IMU_pub.publish(self.IMU_msg)
        # self.pGPS_loc.publish(self.GPS_pos)
