import threading
import time
from ctypes import *

import rospy


class EPOSHandle:
    EPOS_LIB_PATH = '/home/alvaro/Videos/test_ws/src/car_interface/steering/lib/libEposCmd.so.6.8.1.0'
    NodeID = 1

    def __init__(self, max_acc, max_dec, prof_vel):
        self.max_acc = max_acc
        self.max_dec = max_dec
        self.prof_vel = prof_vel
        cdll.LoadLibrary(self.EPOS_LIB_PATH)
        self.epos = CDLL(self.EPOS_LIB_PATH)


        self._is_centered = False # Normal operation shouldn't be before the home position
                                  # has been set, except for the zero_potision_protocol.
        self._is_enabled = False
        self._is_connected = False

    def connect_to_device(self):
        pErrorCode = c_uint()
        if not self._is_connected:
            self.keyhandle = self.epos.VCS_OpenDevice(b'EPOS4', b'MAXON SERIAL V2', b'USB',
                                                      b'USB0', byref(pErrorCode))
            if self.keyhandle == 0:
                rospy.logerr('Failed to connect to EPOS4. Is it on and connected?')
            elif pErrorCode.value != 0:
                rospy.logerr(f'EPOS OpenDevice returned error code {pErrorCode.value}')
            else:
                self._is_connected = True
                rospy.loginfo('EPOS4 Succesfully connected')

            ret = self.epos.VCS_ActivateProfilePositionMode(self.keyhandle, self.NodeID, byref(pErrorCode))

            if ret == 0:
                rospy.logerr(f'EPOS ActivatePositionMode returned error code {pErrorCode.value}')
            else:
                rospy.loginfo(f'Succesfully activated position mode')

            ret = self.epos.VCS_SetPositionProfile(self.keyhandle, self.NodeID, self.prof_vel,
                                                   self.max_acc, self.max_dec, byref(pErrorCode))

            if ret == 0:
                rospy.logerr(f'EPOS SetPositionProfile returned error code {pErrorCode.value}')
            else:
                rospy.loginfo(f'Succesfully set position profile')


    def disconnect_device(self):
        pErrorCode = c_uint()
        if self._is_connected:
            ret = self.epos.VCS_CloseDevice(self.keyhandle, byref(pErrorCode))

            if ret == 0:
                rospy.logerr('Failed to disconnect to EPOS4.')
            elif pErrorCode.value != 0:
                rospy.logerr(f'EPOS CloseDevice returned error code {pErrorCode.value}')
            else:
                self._is_connected = False
                rospy.loginfo('EPOS4 Succesfully disconnected')


    def enable(self):
        pErrorCode = c_uint()
        if not self._is_connected:
            rospy.logerr(f'EPOS4 not connected. Cannot enable')
        elif not self._is_enabled:
            return_code = self.epos.VCS_SetEnableState(self.keyhandle, self.NodeID, byref(pErrorCode))
            if return_code == 0:
                rospy.logerr(f'Couldn\'t enable EPOS4 with error code {pErrorCode.value}')
            else:
                rospy.loginfo('EPOS4 enabled')
                self._is_enabled = True
        else:
            rospy.logwarn('EPOS already enabled')

    def disable(self):
        pErrorCode = c_uint()
        if not self._is_connected:
            rospy.logerr(f'EPOS4 not connected. Cannot disable')
        elif self._is_enabled:
            return_code = self.epos.VCS_SetDisableState(self.keyhandle, self.NodeID, byref(pErrorCode))
            if return_code == 0:
                rospy.logerr(f'Couldn\'t disable EPOS4 with error code {pErrorCode.value}')
            else:
                rospy.loginfo('EPOS4 disabled')
                self._is_enabled = False
        else:
            rospy.logwarn('EPOS already disabled')

    def move_to(self, wheel_angle):
        pErrorCode = c_uint()

        motor_position = int(2*wheel_angle*(2048*5*66/360))#*(180/math.pi))
        if self._is_enabled:
            ret = self.epos.VCS_MoveToPosition(self.keyhandle, self.NodeID, motor_position, 1,
                                               1, byref(pErrorCode))

            if ret == 0:
                rospy.logwarn(f'MoveToPosition error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()

        else:
            rospy.logerr('Cannot move to position with disabled controller :(')

    def get_epos_info(self):
        pErrorCode = c_uint()
        pMovementState = c_uint()
        pPosition = c_float()
        pTargetPosition = c_float()
        pVelocity = c_float()
        pVelocityAvg = c_float()
        pTargetVelocity = c_float()

        pTorque = c_float()
        pTargetTorque = c_float()
        pMotorRatedTorque = c_float()
        pBytesRead = c_uint()

        epos_info = []

        if self._is_enabled:

            ret = self.epos.VCS_GetMovementState(self.keyhandle, self.NodeID, byref(pMovementState), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getMovementState error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            epos_info.append(pMovementState.value)

            ret = self.epos.VCS_GetPositionIs(self.keyhandle, self.NodeID, byref(pPosition), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getPosition error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            epos_info.append(pPosition.value)
            
            ret = self.epos.VCS_GetTargetPosition(self.keyhandle, self.NodeID, byref(pTargetPosition), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getTargetPosition error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            epos_info.append(pTargetPosition.value)
            
            ret = self.epos.VCS_GetVelocityIs(self.keyhandle, self.NodeID, byref(pVelocity), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getVelocity error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            epos_info.append(pVelocity.value)
            
            ret = self.epos.VCS_GetVelocityIsAveraged(self.keyhandle, self.NodeID, byref(pVelocityAvg), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getVelocityAvg error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            epos_info.append(pVelocityAvg.value)

            ret = self.epos.VCS_GetTargetVelocity(self.keyhandle, self.NodeID, byref(pTargetVelocity), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getTargetVelocity error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            epos_info.append(pTargetVelocity.value)

            
            ret = self.epos.VCS_GetObject(self.keyhandle, self.NodeID, 0x6076, 0x00, byref(pMotorRatedTorque), 4, byref(pBytesRead), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getMotorRatedTorque error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()

            ret = self.epos.VCS_GetObject(self.keyhandle, self.NodeID, 0x6077, 0x00, byref(pTorque), 2, byref(pBytesRead), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getTorque error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            #The value is given in per thousand of “Motor rated torque” (uNm)
            epos_info.append((pTorque.value * pMotorRatedTorque.value)/1000)
            
            ret = self.epos.VCS_GetObject(self.keyhandle, self.NodeID, 0x6071, 0x00, byref(pTargetTorque), 2, byref(pBytesRead), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getTargetTorque error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            #The value is given in per thousand of “Motor rated torque” (uNm)
            epos_info.append((pTargetTorque.value * pMotorRatedTorque.value)/1000)
            

            
            return epos_info

        else:
            rospy.logerr('Cannot move to position with disabled controller :(')



    def zero_position_protocol(self):
        pass

    def set_zero_position(self):
        pass


