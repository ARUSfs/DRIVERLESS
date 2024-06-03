import threading
import time
from ctypes import *

import rospy
import rospkg


class EPOSHandle:    
    rospack = rospkg.RosPack()
    EPOS_LIB_PATH = rospack.get_path('steering')+"/lib/libEposCmd.so.6.8.1.0"
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
            #ret = self.epos.VCS_ClearFault(self.keyhandle, self.NodeId, byref(pErrorCode))
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

        motor_position = -int(2*wheel_angle*(2048*5*66/360))#*(180/math.pi))
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
        pPosition = c_long()
        pTargetPosition = c_long()
        pVelocity = c_long()
        pVelocityAvg = c_long()

       
        pTorque = c_int16()
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
            if len(bin(pPosition.value))==34:
                pPosition.value = pPosition.value - 2**32
            epos_info.append(pPosition.value*360/(2*2048*5*66))
            
            ret = self.epos.VCS_GetTargetPosition(self.keyhandle, self.NodeID, byref(pTargetPosition), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getTargetPosition error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            if len(bin(pTargetPosition.value))==34:
                pTargetPosition.value = pTargetPosition.value - 2**32
            epos_info.append(pTargetPosition.value*360/(2*2048*5*66))
            
            ret = self.epos.VCS_GetVelocityIs(self.keyhandle, self.NodeID, byref(pVelocity), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getVelocity error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            if len(bin(pVelocity.value))==34:
                pVelocity.value = pVelocity.value - 2**32
            epos_info.append(pVelocity.value)
            
            ret = self.epos.VCS_GetVelocityIsAveraged(self.keyhandle, self.NodeID, byref(pVelocityAvg), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getVelocityAvg error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            if len(bin(pVelocityAvg.value))==34:
                pVelocityAvg.value = pVelocityAvg.value - 2**32
            epos_info.append(pVelocityAvg.value)


            

            ret = self.epos.VCS_GetObject(self.keyhandle, self.NodeID, 0x6077, 0x00, byref(pTorque), 2, byref(pBytesRead), byref(pErrorCode))
            if ret == 0:
                rospy.logwarn(f'getTorque error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()
            # #The value is given in per thousand of “Motor rated torque” (uNm)
            epos_info.append(pTorque.value)
            
            
            return epos_info

        else:
            rospy.logerr('Cannot get epos info with disabled controller :(')

    def set_position_offset(self,initial_position):
        pErrorCode = c_uint()
        pPositionOffset = c_float()
        pBytesWritten = c_uint()
        pPositionOffset.value = int(2*initial_position*(2048*5*66/360))#*(180/math.pi))
        
        if self._is_enabled:
            ret = self.epos.VCS_SetObject(self.keyhandle, self.NodeID, 0x60B0, 0x00, byref(pPositionOffset), 4, byref(pBytesWritten), byref(pErrorCode))

            if ret == 0:
                rospy.logwarn(f'SetPositionOffset error with code {pErrorCode.value}\n\
                                Disabling controller.')
                self.disable()

        else:
            rospy.logerr('Cannot set position offset with disabled controller :(')

    def zero_position_protocol(self):
        pass

    def set_zero_position(self):
        pass


