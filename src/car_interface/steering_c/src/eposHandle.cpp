#include <stdio.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include "eposHandle.hpp"
#include "Definitions.h"

#define NODE_ID 1

eposHandle::eposHandle(int max_acc, int max_dec, int prof_vel)
{
    this->max_acc = max_acc;
    this->max_dec = max_dec;
    this->prof_vel = prof_vel;
    this->is_connected = false;
    this->is_enabled = false;
    this->is_centered = false;
}

void eposHandle::connect_to_device()
{   
    uint pErrorCode;
    bool ret;
    printf("Connecting to device...\n");
    if(is_connected)
    {
        printf("Device already connected\n");
        return;
    }
    else
    {
        // Connect to device
        g_pKeyHandle = VCS_OpenDevice("EPOS4", "CANopen", "Kvaser", "CAN0", &pErrorCode);
        if(g_pKeyHandle == 0)
        {   
            is_connected = false;
            printf("Device not connected\n");
        }
        else if(pErrorCode != 0)
        {
            printf("Device not connected, error code: %d\n", pErrorCode);
        }
        else
        {
            is_connected = true;
            printf("Device connected\n");
        }

        //Clear fault
        ret = VCS_ClearFault(g_pKeyHandle, NODE_ID, &pErrorCode);
        if(ret)
        {
            printf("Fault cleared\n");
        }
        else
        {
            printf("Fault not cleared\n");
        }

        //Set operation mode
        ret = VCS_ActivateProfilePositionMode(g_pKeyHandle, NODE_ID, &pErrorCode);
        if(ret)
        {
            printf("Profile position mode activated\n");
        }
        else
        {
            printf("EPOS ActivatePositionMode returned error code: %d\n", pErrorCode);
        }

        //Set position profile
        ret = VCS_SetPositionProfile(g_pKeyHandle, NODE_ID, prof_vel, max_acc, max_dec, &pErrorCode);
        if(ret)
        {
            printf("Position profile set\n");
        }
        else
        {
            printf("EPOS SetPositionProfile returned error code: %d\n", pErrorCode);
        }
    }
    
}