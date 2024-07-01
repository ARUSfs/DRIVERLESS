#include <canlib.h>
#include <stdio.h>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <math.h>
#include<iostream>
#include <cstdint>
#include <cstring>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <common_msgs/Controls.h>
#include "canInterface.hpp"



int velMax = 5500;
float wheelRadius = 0.2;
float transmissionRatio = 0.24444444444444444;//11/45;


void CanInterface::check_can(canStatus stat)
{
    if(stat != canOK){
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("failed, stat=%d (%s)\n", (int)stat, buf);
    }
}

//################################################# CAN HANDLE ############################################################
CanInterface::CanInterface()
{    

     // Subscribers
    controlsSub = nh.subscribe<common_msgs::Controls>("/controls", 100, &CanInterface::controlsCallback, this);    
    ASStatusSub = nh.subscribe<std_msgs::Int16>("can/AS_status", 100, &CanInterface::ASStatusCallback, this);
    steeringInfoSub = nh.subscribe<std_msgs::Int32MultiArray>("/steering/epos_info", 100, &CanInterface::steeringInfoCallback, this);

    // Publishers
    motorSpeedPub = nh.advertise<std_msgs::Float32>("/motor_speed", 100);
    ASStatusPub = nh.advertise<std_msgs::Int16>("/can/AS_status", 100);
    GPSPub = nh.advertise<sensor_msgs::NavSatFix>("can/gps", 100);
    GPSSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/gps_speed", 100);
    IMUPub = nh.advertise<sensor_msgs::Imu>("can/IMU", 100);
    steeringAnglePub = nh.advertise<std_msgs::Float32>("can/steering_angle", 100);
    rearWheelSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/rear_wheel_speed", 100);
    frontWheelSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/front_wheel_speed", 100);
    RESRangePub = nh.advertise<std_msgs::Float32>("/can/RESRange", 100);

    //Timers
    canInitializeLibrary(); // Initialize the library
    std::cout << "Librería inicializada" << std::endl;

    canStatus stat;
    int chanCount;
    stat = canGetNumberOfChannels(&chanCount);
    printf("%d channels.\n", chanCount);

    std::thread thread_0(&CanInterface::readCan0, this);
    std::thread thread_1(&CanInterface::readCan1, this);

    hndW0 = canOpenChannel(0, canOPEN_ACCEPT_VIRTUAL);
    if (hndW0 < 0){
        printf("canOpenChannel() failed, %d\n", hndW0);
        return;
    } else{
        printf("can0 enabled for writing");
    }

    hndW1 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hndW1 < 0){
        printf("canOpenChannel() failed, %d\n", hndW1);
        return;
    }else{
        printf("can1 enabled for writing");
    }

    canBusOn(hndW0);
    canBusOn(hndW1);

    ros::waitForShutdown();

    thread_0.join();
    thread_1.join();

}

//################################################# PARSE FUNCTIONS #################################################

//--------------------------------------------- INV SPEED -------------------------------------------------------------
void CanInterface::parseInvSpeed(uint8_t msg[8])
{      
    int16_t val = (msg[2] << 8) | msg[1];
    float angV = val / pow(2, 15) * velMax;
    float invSpeed = -angV * 2 * M_PI * wheelRadius * transmissionRatio / 60;
    std_msgs::Float32 x;
    x.data = invSpeed;
    this->motorSpeedPub.publish(x);
}

//-------------------------------------------- AS -------------------------------------------------------------------------
void CanInterface::parseASStatus(uint8_t msg[8])
{
    int16_t val = (msg[2]);
    std_msgs::Int16 x;
    x.data = val;
    this->ASStatusPub.publish(x);
}

//-------------------------------------------- IMU -----------------------------------------------------------------------
void CanInterface::parseAcc(uint8_t msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float accX = intX*0.01;

    int16_t intY = (msg[3] << 8) | msg[2];
    float accY = intY*0.01;

    int16_t intZ = (msg[5] << 8) | msg[4];
    float accZ = intZ*0.01;


    IMUData.linear_acceleration.x = accX;
    IMUData.linear_acceleration.y = accY;
    IMUData.linear_acceleration.z = accZ;
    IMUData.header.stamp = ros::Time::now();  

    this->IMUPub.publish(IMUData);  
}

void CanInterface::parseEulerAngles(uint8_t msg[8])
{
    int16_t intRoll = (msg[1] << 8) | msg[0];
    float roll = intRoll*0.0001;

    int16_t intPitch = (msg[3] << 8) | msg[2];
    float pitch = intPitch*0.0001;

    int16_t intYaw = (msg[5] << 8) | msg[4];
    float yaw = intYaw*0.0001;

    float qx = sin(roll*0.5) * cos(pitch/2) * cos(yaw*0.5) - cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
    float qy = cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5);
    float qz = cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5) - sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5);
    float qw = cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);

    IMUData.orientation.x = qx;
    IMUData.orientation.y = qy;
    IMUData.orientation.z = qz;
    IMUData.orientation.w = qw;
    IMUData.header.stamp = ros::Time::now();    
}

void CanInterface::parseAngularVelocity(uint8_t msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float angVelX = intX*0.001;

    int16_t intY = (msg[3] << 8) | msg[2];
    float angVelY = intY*0.001;

    int16_t intZ = (msg[5] << 8) | msg[4];
    float angVelZ = intZ*0.001;

    IMUData.angular_velocity.x = angVelX;
    IMUData.angular_velocity.y = angVelY;
    IMUData.angular_velocity.z = angVelZ;
    IMUData.header.stamp = ros::Time::now();    
}

void CanInterface::parseGPS(uint8_t msg[8])
{
    int32_t lat = (msg[3] << 24) | (msg[2] << 16) | (msg[1] << 8) | msg[0];
    int32_t lon = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];

    sensor_msgs::NavSatFix x;
    x.latitude = lat;
    x.longitude = lon;
    this->GPSPub.publish(x);
}

void CanInterface::parseGPSVel(uint8_t msg[8])
{
    int16_t inxN = (msg[1] << 8) | msg[0];
    float velN = inxN/100;

    int16_t inxE = (msg[3] << 8) | msg[2];
    float velE = inxE/100;

    int16_t inxD = (msg[5] << 8) | msg[4];
    float velD = inxD/100;

    geometry_msgs::Vector3 x;
    x.x = velN;
    x.y = velE;
    x.z = velD;
    this->GPSSpeedPub.publish(x);
}

//-------------------------------------------------------- ACQUISITION -------------------------------------------------
void CanInterface::parseSteeringAngle(uint8_t msg[8])
{
    int16_t val = (msg[2] << 8) | msg[1];

    std_msgs::Float32 x;
    x.data = val;
    this->steeringAnglePub.publish(x);
}

void CanInterface::parseFrontWheelSpeed(uint8_t msg[8])
{
    int16_t intLeft = (msg[2] << 8) | msg[1];

    int16_t intRight = (msg[4] << 8) | msg[3];

    geometry_msgs::Vector3 x;
    x.x = intLeft;
    x.y = intRight;

    this->frontWheelSpeedPub.publish(x);
}

void CanInterface::parseRearWheelSpeed(uint8_t msg[8])
{
    int16_t intLeft = (msg[2] << 8) | msg[1];

    int16_t intRight = (msg[4] << 8) | msg[3];

    geometry_msgs::Vector3 x;
    x.x = intLeft;
    x.y = intRight;

    this->rearWheelSpeedPub.publish(x);
}

//---------------------------------------------RES---------------------------------------------------------------
void CanInterface::parseRES(uint8_t msg[8])
{
    uint8_t val = msg[6];
    float perc = (static_cast<float>(val) / 255.0) * 100.0;

    std_msgs::Float32 x;
    x.data = perc;
    this->RESRangePub.publish(x);
}

//################################################# READ FUNCTIONS #################################################

//--------------------------------------------- CAN 0 -------------------------------------------------------------------   
void CanInterface::readCan0()
{   
    canStatus stat;

    // Open handle to channel 0
    CanHandle hndR0 = canOpenChannel(0, canOPEN_ACCEPT_VIRTUAL);
    if (hndR0 < 0){
        printf("canOpenChannel() failed, %d\n", hndR0);
        return;
    }else{
        printf("can0 enabled for reading");
    }

    //Set the channel parameters
    //stat = canSetAcceptanceFilter(hndR0, 0x181, 0x7FF, 0);
    //CanInterface::check_can(stat);

    //stat = canSetAcceptanceFilter(hndR0, 0x18b, 0x7FF, 0);
    //CanInterface::check_can(stat);

    stat = canBusOn(hndR0);
    CanInterface::check_can(stat);

    //Read
    while (true){

        long id;
        uint8_t msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        stat = canReadWait(hndR0, &id, &msg, &dlc, &flag, &time, 300);
        int8_t subId = msg[0];

        

        if(stat == canOK){
            switch(id){
                case 0x181:
                    
                    if(subId == 0x30) parseInvSpeed(msg);  
                    break;
                case 0x182:
                    switch(subId){
                        case 0x01:
                            parseASStatus(msg);
                            break;
                        case 0x04:
                            parseFrontWheelSpeed(msg);
                            break;
                        case 0x05:
                            parseRearWheelSpeed(msg);
                            break;
                        default:
                            break;
                    }
                case 0x18B:
                    parseRES(msg);
                    break;
                default:
                    break;
            }
        }

        else if(stat = canERR_NOMSG){
            uint8_t data[3] = {0x01, 0x01, 0x04};
            canWrite(hndR0, 0x202, data, 3, canMSG_STD);
        }
    }

    stat = canBusOff(hndR0);
    stat = canClose(hndR0);
}

//--------------------------------------------- CAN 1 -------------------------------------------------------------------
void setFilter(CanHandle hnd, int code){

    canStatus stat;
    stat = canSetAcceptanceFilter(hnd, code, 0x7FF, 0);
    CanInterface::check_can(stat);
}

void CanInterface::readCan1()
{   
    canStatus stat;

    // Open handle to channel 1
    CanHandle hndR1 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hndR1 < 0){
        printf("canOpenChannel() failed, %d\n", hndR1);
        return;
    }else{
        printf("can1 enabled for reading");
    }

    //Set the channel parameters
    // setFilter(hndR1, 0x182);
    // setFilter(hndR1, 0x380);
    // setFilter(hndR1, 0x394);
    // setFilter(hndR1, 0x392);
    // setFilter(hndR1, 0x384);
    // setFilter(hndR1, 0x382);
    // setFilter(hndR1, 0x185);
    // setFilter(hndR1, 0x205);
    // setFilter(hndR1, 0x334);
    // setFilter(hndR1, 0x187);

    stat = canBusOn(hndR1);
    //Read
    while (true){

        long id;
        uint8_t msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        stat = canReadWait(hndR1, &id, &msg, &dlc, &flag, &time, 300);
        uint8_t subId = msg[0];


        if (stat == canOK){  
            switch(id)
            {
                case 0x380:
                    parseAcc(msg);
                    break;
                case 0x394:
                    parseGPS(msg);
                    break;
                case 0x392:
                    parseGPSVel(msg);
                    break;
                case 0x384:
                    parseEulerAngles(msg);
                    break;
                case 0x382:
                    parseAngularVelocity(msg);
                    break;
                case 0x187:
                    switch(subId)
                    {
                        case 0x01:
                            parseSteeringAngle(msg);
                            break;
                    }
                    break;
                default:
                    break;
            }
        }

        else if(stat = canERR_NOMSG){
            uint8_t data[3] = {0x01, 0x01, 0x04};
            canWrite(hndR1, 0x202, data, 3, canMSG_STD);
        }
    }

    stat = canBusOff(hndR1);
    stat = canClose(hndR1);
}

//################################################# CALLBACKS ###########################################################
void intToBytes(int16_t val, int8_t* bytes)
{
    std::memcpy(bytes, &val, sizeof(val));
}           

void CanInterface::controlsCallback(common_msgs::Controls msg)
{
    // std::cout << "llega" << std::endl;
    float acc = msg.accelerator;
    int16_t intValue = static_cast<int16_t>(acc * (1<<15))-1;

    int8_t bytesCMD[2];
    intToBytes(intValue, bytesCMD);
    int8_t cabecera = 0x90;

    int8_t data[3] = {cabecera, bytesCMD[0], bytesCMD[1]};

    canWrite(hndW0, 0x201, data, 3, canMSG_STD);
}

void CanInterface::ASStatusCallback(std_msgs::Int16 msg)
{
    if(msg.data == 3){
        uint8_t data[3] = {0x01, 0x01, 0x03};
        canWrite(hndW1, 0x202, data, 8, canMSG_STD);
    }
}

void CanInterface::steeringInfoCallback(std_msgs::Int32MultiArray msg)
{
    int8_t pMovementState = msg.data[0];

    int16_t pPosition = msg.data[1]*100;
    int8_t pPositionBytes[3];
    intToBytes(pPosition, pPositionBytes);

    int16_t pTargetPosition = msg.data[2]*100;
    int8_t pTargetPositionBytes[3];
    intToBytes(pTargetPosition, pTargetPositionBytes);

    int8_t msgEposState[8]= {0x02, pMovementState, pPositionBytes[0], pPositionBytes[1], pPositionBytes[2], pTargetPositionBytes[0], pTargetPositionBytes[1], pTargetPositionBytes[2]};
    canWrite(hndW1, 0x183, msgEposState, 8, canMSG_STD);

    int16_t pVelocity = msg.data[3]*100;
    int8_t pVelocityBytes[3];
    intToBytes(pVelocity, pVelocityBytes);

    int16_t pVelocityAvg = msg.data[4]*100;
    int8_t pVelocityAvgBytes[3];
    intToBytes(pVelocityAvg, pVelocityAvgBytes);

    int8_t msgEposVelocity[7]= {0x03, pVelocityBytes[0], pVelocityBytes[1], pVelocityBytes[2], pVelocityAvgBytes[0], pVelocityAvgBytes[1], pVelocityAvgBytes[2]};
    canWrite(hndW1, 0x183, msgEposVelocity, 7, canMSG_STD);

    int16_t pTorque = msg.data[5]*100;
    int8_t pTorqueBytes[2];
    intToBytes(pTorque, pTorqueBytes);
    
    int8_t msgEposTorque[3]= {0x04, pTorqueBytes[0], pTorqueBytes[1]};
    canWrite(hndW1, 0x183, msgEposTorque, 3, canMSG_STD);
}



void CanInterface::pubHeartBeat(const ros::TimerEvent&)
{
    uint8_t data[1] = {0x01};
    canWrite(hndW1, 0x183, data, 1, canMSG_STD);
}
