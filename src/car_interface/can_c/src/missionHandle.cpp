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
canHandle hnd2;
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

    hnd2 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hnd2 < 0)
    {
        printf("canOpenChannel2 failed, status=%d\n", hnd2);
    }

    //Oneamos el bus
    stat = canBusOn(hnd);
    if (stat != canOK)
    {
        printf("canBusOn failed, status=%d\n", stat);
    }

    stat = canBusOn(hnd2);
    if (stat != canOK)
    {
        printf("canBusOn failed 2, status=%d\n", stat);
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

        stat = canBusOff(hnd2);
        if (stat != canOK)
        {
            printf("canBusOff failed, status=%d\n", stat);
        }

        stat = canClose(hnd2);
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
            baseCommand += "trackdrive.launch";
            break;
        case 3:
            baseCommand += "skidpad.launch";
            break;
        case 4:
            baseCommand += "autocross.launch";
            break;
        case 5:
            baseCommand += "ebs_test.launch";
            break;
        case 6:
            baseCommand += "inspection.launch";
            break;
        default:
            break;
    }


    int ret = system(baseCommand.c_str());

    if(ret == 0)
    {
        ROS_INFO("Mission launched successfully\n");
    }
    else
    {
        ROS_INFO("Mission failed to launch\n");
    }
}

void pubHeartBeat(const ros::TimerEvent&)
{
    uint8_t data[1] = {0x00};
    canWrite(hnd2, 0x183, data, 1, canMSG_STD);
}

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"carlos_mpc");
    ros::NodeHandle nh;

    ros::Timer heartBeatTimer = nh.createTimer(ros::Duration(0.1), pubHeartBeat);


    initCan();

    while(ros::ok())
    {
        ros::spinOnce();
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
                ROS_INFO("HV ON \n");
                }
            }
            else
            {
                if((id == 0x185) && (msg[0]==0x01))
                {
                    ROS_INFO("Mission received\n");
                    mission = msg[1];
                    closeCan();
                    launchMission();
                    while(true){}
                   break;
                }
            }
        }
    }
}
