#include <cstdint>
#include <cstring>
#include <canlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <ros/ros.h>


bool HV_ON = false;
canStatus stat;
canHandle hnd;
uint8_t mission;
std::string baseCommand = "roslaunch common_meta ";

void initCan()
{   
    //Inicializamos la libreria
    canInitializeLibrary();

    //Openeamos el canalw
    hnd = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0)
    {
        printf("canOpenChannel failed, status=%d\n", hnd);
    }

    //Oneamos el bus
    stat = canBusOn(hnd);
    if (stat != canOK)
    {
        printf("canBusOn failed, status=%d\n", stat);
    }
}

void closeCan()
{
        stat = canBusOff(hnd);
        if (stat != canOK)
        {
            printf("canBusOff failed, status=%d\n", stat);
        }

        stat = canClose(hnd);
        if (stat != canOK)
        {
            printf("canClose failed, status=%d\n", stat);
        }

        canUnloadLibrary();    
}

void launchMission()
{
    switch(mission)
    {
        case 1:
            baseCommand += "acceleration.launch";
            break;
        case 2:
            baseCommand += "skidpad.launch";
            break;
        case 3:
            baseCommand += "autocross.launch";
            break;
        case 4:
            baseCommand += "trackdrive.launch";
            break;
        case 5:
            baseCommand += "EBS_test.launch";
            break;
        case 6:
            baseCommand += "inspection.launch";
            break;
        default:
            break;
    }

    baseCommand += " &";

    int ret = system(baseCommand.c_str());

    if(ret == 0)
    {
        printf("Mission launched successfully\n");
    }
    else
    {
        printf("Mission failed to launch\n");
    }
}

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"carlos_mpc");
    ros::NodeHandle n;

    initCan();

    while(ros::ok())
    {
        long id;
        uint8_t msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;

        stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);

        if (stat == canOK)
        {
            if(!HV_ON)
            {
                if((id == 0x186) && (msg[0] == 0x00) && (msg[1] == 0x0F)){
                 HV_ON = true; //Se da alta
                ROS_INFO("HV ON \n")
                }
            }
            else
            {
                if((id == 0x185) && (msg[0]==0x01))
                {
                    ROS_INFO("Mission received\n")
                    mission = msg[1];
                    closeCan();
                    launchMission();
                   break;
                }
            }
        }
    }
}
