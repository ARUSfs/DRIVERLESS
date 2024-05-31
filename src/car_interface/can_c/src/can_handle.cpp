#include <canlib.h>
#include <stdio.h>
#include <thread>
#include <ros/ros.h>
#include <thread>
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

#include "can_handle.hpp"

using namespace std;


int velMax = 5500;
float wheelRadius = 0.2;
float transmissionRatio = 11/45;


//################################################# PARSE FUNCTIONS #################################################

//--------------------------------------------- INV SPEED -------------------------------------------------------------
void CanInterface::parseInvSpeed(unsigned char msg[8])
{
    int16_t val = (msg[2] << 8) | msg[1];
    float angV = val / pow(2, 15) * velMax;
    float invSpeed = -angV * 2 * M_PI * wheelRadius * transmissionRatio / 60;

    std_msgs::Float32 x;
    x.data = invSpeed;
    motorSpeedPub.publish(x);
}

//-------------------------------------------- AS -------------------------------------------------------------------------
void CanInterface::parseASStatus(unsigned char msg[8])
{
    int16_t val = (msg[2]);
    std_msgs::Int16 x;
    x.data = val;
    ASStatusPub.publish(x);
}

//-------------------------------------------- IMU -----------------------------------------------------------------------
void CanInterface::parseAcc(unsigned char msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float accX = intX/100;

    int16_t intY = (msg[3] << 8) | msg[2];
    float accY = intY/100;

    int16_t intZ = (msg[5] << 8) | msg[4];
    float accZ = intZ/100;

    CanInterface::IMUData.linear_acceleration.x = accX;
    CanInterface::IMUData.linear_acceleration.y = accY;
    CanInterface::IMUData.linear_acceleration.z = accZ;
    CanInterface::IMUData.header.stamp = ros::Time::now();    
}

void CanInterface::parseEulerAngles(unsigned char msg[8])
{
    int16_t intRoll = (msg[1] << 8) | msg[0];
    float roll = intRoll/10000;

    int16_t intPitch = (msg[3] << 8) | msg[2];
    float pitch = intPitch/10000;

    int16_t intYaw = (msg[5] << 8) | msg[4];
    float yaw = intYaw/10000;

    float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

    CanInterface::IMUData.orientation.x = qx;
    CanInterface::IMUData.orientation.y = qy;
    CanInterface::IMUData.orientation.z = qz;
    CanInterface::IMUData.orientation.w = qw;
    CanInterface::IMUData.header.stamp = ros::Time::now();    
}

void CanInterface::parseAngularVelocity(unsigned char msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float angVelX = intX/1000;

    int16_t intY = (msg[3] << 8) | msg[2];
    float angVelY = intY/1000;

    int16_t intZ = (msg[5] << 8) | msg[4];
    float angVelZ = intZ/1000;

    CanInterface::IMUData.angular_velocity.x = angVelX;
    CanInterface::IMUData.angular_velocity.y = angVelY;
    CanInterface::IMUData.angular_velocity.z = angVelZ;
    CanInterface::IMUData.header.stamp = ros::Time::now();    
}

void CanInterface::parseGPS(unsigned char msg[8])
{
    int32_t lat = (msg[3] << 24) | (msg[2] << 16) | (msg[1] << 8) | msg[0];
    int32_t lon = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];

    sensor_msgs::NavSatFix x;
    x.latitude = lat;
    x.longitude = lon;
    GPSPub.publish(x);
}

void CanInterface::parseGPSVel(unsigned char msg[8])
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
    GPSSpeedPub.publish(x);
}

//-------------------------------------------------------- ACQUISITION -------------------------------------------------
void CanInterface::parseSteeringAngle(unsigned char msg[8])
{
    int16_t val = (msg[2] << 8) | msg[1];

    std_msgs::Float32 x;
    x.data = val;
    steeringAnglePub.publish(x);
}

void CanInterface::parseFrontWheelSpeed(unsigned char msg[8])
{
    int16_t intLeft = (msg[2] << 8) | msg[1];

    int16_t intRight = (msg[4] << 8) | msg[3];

    geometry_msgs::Vector3 x;
    x.x = intLeft;
    x.y = intRight;

    frontWheelSpeedPub.publish(x);
}

void CanInterface::parseRearWheelSpeed(unsigned char msg[8])
{
    int16_t intLeft = (msg[2] << 8) | msg[1];

    int16_t intRight = (msg[4] << 8) | msg[3];

    geometry_msgs::Vector3 x;
    x.x = intLeft;
    x.y = intRight;

    rearWheelSpeedPub.publish(x);
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
    }

    // Set the channel parameters
    stat = canSetAcceptanceFilter(hndR0, 0x181, 0x7FF, 0);
    if (stat < 0){
        printf("canSetAcceptanceFilter() failed, %d\n", stat);    
    }

    stat = canSetAcceptanceFilter(hndR0, 0x18b, 0x7FF, 0);
    if (stat < 0){
        printf("canSetAcceptanceFilter() failed, %d\n", stat);    
    }

    stat = canBusOn(hndR0);

    //Read
    while (true){
        long id;
        unsigned char msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        stat = canRead(hndR0, &id, &msg, &dlc, &flag, &time);
        unsigned char subId = msg[0];

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
                default:
            break;
            }
        }
    }

    stat = canBusOff(hndR0);
    stat = canClose(hndR0);
}

//--------------------------------------------- CAN 1 -------------------------------------------------------------------
void setFilter(CanHandle hnd, int code){
    canStatus stat;

    stat = canSetAcceptanceFilter(hnd, code, 0x7FF, 0);
    if (stat < 0){
        printf("canSetAcceptanceFilter() failed, %d\n", stat);    
    }
}

void CanInterface::readCan1()
{
    canStatus stat;

    // Open handle to channel 1
    CanHandle hndR1 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hndR1 < 0){
        printf("canOpenChannel() failed, %d\n", hndR1);
        return;
    }

    // Set the channel parameters
    setFilter(hndR1, 0x182);
    setFilter(hndR1, 0x380);
    setFilter(hndR1, 0x394);
    setFilter(hndR1, 0x392);
    setFilter(hndR1, 0x384);
    setFilter(hndR1, 0x382);
    setFilter(hndR1, 0x185);
    setFilter(hndR1, 0x205);
    setFilter(hndR1, 0x334);
    setFilter(hndR1, 0x187);

    stat = canBusOn(hndR1);
    //Read
    while (true){
        long id;
        unsigned char msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        stat = canRead(hndR1, &id, &msg, &dlc, &flag, &time);
        unsigned char subId = msg[0];


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

    float acc = msg.accelerator;
    int16_t intValue = static_cast<int16_t>(acc * (1<<15))-1;

    int8_t bytesCMD[2];
    intToBytes(intValue, bytesCMD);
    char cabecera = 0x90;

    signed char data[3] = {cabecera, bytesCMD[0], bytesCMD[1]};

    canWrite(hndW0, 0x201, data, 3, canMSG_STD);
}

void CanInterface::ASStatusCallback(std_msgs::Int16 msg)
{
    if(msg.data == 3){
        char data[3] = {0x01, 0x01, 0x03};
        canWrite(hndW1, 0x202, data, 8, canMSG_STD);
    }
}

void CanInterface::steeringInfoCallback(std_msgs::Int32MultiArray msg)
{
    char pMovementState = msg.data[0];

    int16_t pPosition = msg.data[1]*100;
    int8_t pPositionBytes[3];
    intToBytes(pPosition, pPositionBytes);

    int16_t pTargetPosition = msg.data[2]*100;
    int8_t pTargetPositionBytes[3];
    intToBytes(pTargetPosition, pTargetPositionBytes);

    char msgEposState[8]= {0x02, pMovementState, pPositionBytes[0], pPositionBytes[1], pPositionBytes[2], pTargetPositionBytes[0], pTargetPositionBytes[1], pTargetPositionBytes[2]};
    canWrite(hndW1, 0x183, msgEposState, 8, canMSG_STD);

    int16_t pVelocity = msg.data[3]*100;
    int8_t pVelocityBytes[3];
    intToBytes(pVelocity, pVelocityBytes);

    int16_t pVelocityAvg = msg.data[4]*100;
    int8_t pVelocityAvgBytes[3];
    intToBytes(pVelocityAvg, pVelocityAvgBytes);

    char msgEposVelocity[7]= {0x03, pVelocityBytes[0], pVelocityBytes[1], pVelocityBytes[2], pVelocityAvgBytes[0], pVelocityAvgBytes[1], pVelocityAvgBytes[2]};
    canWrite(hndW1, 0x183, msgEposVelocity, 7, canMSG_STD);

    int16_t pTorque = msg.data[5]*100;
    int8_t pTorqueBytes[2];
    intToBytes(pTorque, pTorqueBytes);
    
    char msgEposTorque[3]= {0x04, pTorqueBytes[0], pTorqueBytes[1]};
    canWrite(hndW1, 0x183, msgEposTorque, 3, canMSG_STD);
}

//################################################# CAN HANDLE ############################################################
CanInterface::CanInterface()
{
    canInitializeLibrary(); // Initialize the library
    std::thread thread_0(&CanInterface::readCan0, this);
    std::thread thread_1(&CanInterface::readCan1, this);

    hndW0 = canOpenChannel(0, canOPEN_ACCEPT_VIRTUAL);
    if (hndW0 < 0){
        printf("canOpenChannel() failed, %d\n", hndW0);
        return;
    }

    hndW1 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hndW1 < 0){
        printf("canOpenChannel() failed, %d\n", hndW1);
        return;
    }

    canBusOn(hndW0);
    canBusOn(hndW1);

    // Publishers
    motorSpeedPub = nh.advertise<std_msgs::Float32>("can/motor_speed", 100);
    ASStatusPub = nh.advertise<std_msgs::Int16>("can/AS_status", 100);
    GPSPub = nh.advertise<sensor_msgs::NavSatFix>("can/gps", 100);
    GPSSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/gps_speed", 100);
    IMUPub = nh.advertise<sensor_msgs::Imu>("can/imu", 100);
    steeringAnglePub = nh.advertise<std_msgs::Float32>("can/steering_angle", 100);
    rearWheelSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/rear_wheel_speed", 100);
    frontWheelSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/front_wheel_speed", 100);

    // Subscribers
    controlsSub = nh.subscribe<common_msgs::Controls>("/controls", 100, &CanInterface::controlsCallback, this);    
    ASStatusSub = nh.subscribe<std_msgs::Int16>("can/AS_status", 100, &CanInterface::ASStatusCallback, this);
    steeringInfoSub = nh.subscribe<std_msgs::Int32MultiArray>("can/steering_info", 100, &CanInterface::steeringInfoCallback, this);
}

