#include <canlib.h>
#include <stdio.h>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include<iostream>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include "sensor_msgs/PointCloud2.h"
#include <common_msgs/Controls.h>
#include "canInterface.hpp"
#include <sstream>
#include <stdlib.h>

int velMax = 5500;
float wheelRadius = 0.2;
float transmissionRatio = 0.24444444444444444;//11/45;
 
void CanInterface::check_can(canStatus stat)
{
    if(stat != canOK){
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("failed, stat=%d (%s) \n", (int)stat, buf);
    }
}

//################################################# CAN HANDLE ############################################################
CanInterface::CanInterface()
{    
     // Subscribers
    controlsSub = nh.subscribe<common_msgs::Controls>("/controls", 100, &CanInterface::controlsCallback, this);    
    ASStatusSub = nh.subscribe<std_msgs::Int16>("/can/AS_status", 100, &CanInterface::ASStatusCallback, this);
    steeringInfoSub = nh.subscribe<std_msgs::Float32MultiArray>("/steering/epos_info", 100, &CanInterface::steeringInfoCallback, this);
    lapCounterSub = nh.subscribe<std_msgs::Int16>("/lap_counter", 100, &CanInterface::lapCounterCallback, this);
    conesCountSub = nh.subscribe<sensor_msgs::PointCloud2>("/perception_map", 100, &CanInterface::conesCountCallback, this);
    conesCountAllSub = nh.subscribe<sensor_msgs::PointCloud2>("/mapa_icp", 100, &CanInterface::conesCountAllCallback, this);
    targetSpeedSub = nh.subscribe<std_msgs::Int16>("/target_speed", 100, &CanInterface::targetSpeedCallback, this);
    brakeLightSub = nh.subscribe<std_msgs::Int16>("/brake_light", 100, &CanInterface::brakeLightCallback, this);

    // Publishers
    motorSpeedPub = nh.advertise<std_msgs::Float32>("/motor_speed", 100);
    ASStatusPub = nh.advertise<std_msgs::Int16>("/can/AS_status", 100);
    GPSPub = nh.advertise<sensor_msgs::NavSatFix>("can/gps", 100);
    GPSSpeedPub = nh.advertise<geometry_msgs::Vector3>("can/gps_speed", 100);
    IMUPub = nh.advertise<sensor_msgs::Imu>("can/IMU", 100);
    steeringAnglePub = nh.advertise<std_msgs::Float32>("can/steering_angle", 100);
    RESRangePub = nh.advertise<std_msgs::Float32>("/can/RESRange", 100);
    PCTempPub = nh.advertise<std_msgs::Float32>("/pc_temp", 100);
    DL500Pub = nh.advertise<std_msgs::Float32MultiArray>("/can/DL500", 100);
    DL501Pub = nh.advertise<std_msgs::Float32MultiArray>("/can/DL501", 100);
    DL502Pub = nh.advertise<std_msgs::Float32MultiArray>("/can/DL502", 100);

    //Timers
    pcTempTimer = nh.createTimer(ros::Duration(0.1), &CanInterface::pcTempCallback, this);
    heartBeatTimer = nh.createTimer(ros::Duration(0.1), &CanInterface::pubHeartBeat, this);
    DL500Timer = nh.createTimer(ros::Duration(0.1), &CanInterface::DL500Callback, this);
    DL501Timer = nh.createTimer(ros::Duration(0.1), &CanInterface::DL501Callback, this);
    DL502Timer = nh.createTimer(ros::Duration(0.1), &CanInterface::DL502Callback, this);

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
        printf("can0 enabled for writing \n");
    }

    stat = canSetBusParams(hndW0, canBITRATE_1M,0,0,0,0,0);
    if (stat!=canOK){
        printf("canSetBusParams failed, status=%d\n", stat);
        exit(1);
    }

    hndW1 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hndW1 < 0){
        printf("canOpenChannel() failed, %d \n", hndW1);
        return;
    }else{
        printf("can1 enabled for writing \n");
    }

    canBusOn(hndW0);
    canBusOn(hndW1);

    ros::waitForShutdown();

    thread_0.join();
    thread_1.join();

    this->brake_hydr_actual = 100;
    this->brake_hydr_target = 100;
    this->service_brake_state = 0;
    this->cones_count_all = 0;
    this->EBS_state = 0;
    this->motor_moment_actual = 0;
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
    this->actual_speed = (invSpeed*1000)/3600;
}

//-------------------------------------------- AS -------------------------------------------------------------------------
//1111
//133
void CanInterface::parseASStatus(uint8_t msg[8])
{
    int16_t val = (msg[2]);

    if(val == 0x02)
    {
        this->brake_hydr_actual = 0;
        this->brake_hydr_target = 0;
    }else{
        this->brake_hydr_actual = 100;
        this->brake_hydr_target = 100;
    }

    this->AS_state = val+1;

    std_msgs::Int16 x;
    x.data = val;
    this->ASStatusPub.publish(x);
}

void CanInterface::parseBrakeHydr(uint8_t msg[8])
{
    uint16_t b = (msg[2]<<8) | msg[1];
    this -> brake_hydr_actual = ((b-1111)/(133-1111))*100;
}

//-------------------------------------------- IMU -----------------------------------------------------------------------
void CanInterface::parseAcc(uint8_t msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float accX = intX*0.01;

    int16_t intY = (msg[3]  << 8) | msg[2];
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

    this->actual_steering_angle = val * 2;

    std_msgs::Float32 x;
    x.data = val;
    this->steeringAnglePub.publish(x);
}


//---------------------------------------------RES---------------------------------------------------------------
void CanInterface::parseRES(uint8_t msg[8])
{
    uint8_t val = msg[6];
    std_msgs::Float32 x;
    x.data = val;
    this->RESRangePub.publish(x);
}

//---------------------------------------------DASHBOARD---------------------------------------------------------------
void CanInterface::parseMission(uint8_t msg[8])
{
    uint8_t val = msg[1];

    switch (val)
    {
    case 0x01:
        this->AMI_state = 1;
        break;
    case 0x02:
        this->AMI_state = 2;
        break;
    case 0x03:
        this->AMI_state = 6;
        break;
    case 0x04:
        this->AMI_state = 3;
        break;
    case 0x05:
        this->AMI_state = 4;
        break;
    case 0x06:
        this->AMI_state = 5;
        break;
    default:
        break;
    }
}

//################################################# READ FUNCTIONS #################################################

//--------------------------------------------- CAN 0 -------------------------------------------------------------------   
void CanInterface::readCan0()
{   
    canStatus stat;

    // Open handle to channel 0
    CanHandle hndR0 = canOpenChannel(0, canOPEN_ACCEPT_VIRTUAL);
    if (hndR0 < 0){
        printf("canOpenChannel() failed, %d \n", hndR0);
        return;
    }else{
        printf("can0 enabled for reading \n");
    }

    stat = canSetBusParams(hndR0, canBITRATE_1M,0,0,0,0,0);
    if (stat!=canOK){
        printf("canSetBusParams failed, status=%d\n", stat);
        exit(1);
    }
    

    // //Set the channel parameters
    // stat = canAccept(hndR0, 0xFFF, canFILTER_SET_MASK_STD);
    // CanInterface::check_can(stat);
    // stat = canAccept(hndR0, 0x181, canFILTER_SET_CODE_STD);
    // CanInterface::check_can(stat);
    // stat = canAccept(hndR0, 0x18B, canFILTER_SET_CODE_STD);
    // CanInterface::check_can(stat);

    stat = canBusOn(hndR0);
    CanInterface::check_can(stat);

    //Read
    while (ros::ok()){

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
                        case 0x04: //Brake pressure
                            parseBrakeHydr(msg);
                            break; 
                        case 0x05: //Pneumtic pressure
                            break;
                        case 0x06: //Valves state
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
    stat = canAccept(hnd, code, canFILTER_SET_CODE_STD);
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
        printf("can1 enabled for reading \n");
    }

    // //Set the channel parameters
    // stat = canAccept(hndR1, 0xFFF, canFILTER_SET_MASK_STD);
    // CanInterface::check_can(stat);

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
    while (ros::ok()){

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
                        case 0x04:
                            break;
                    }
                    break;
                case 0x185:
                    switch(subId)
                    {
                        case 0x01:
                            parseMission(msg);
                    }
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
    this->motor_moment_target = intValue;

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
        canWrite(hndW0, 0x202, data, 3, canMSG_STD);
    }else if(msg.data==4){
        uint8_t data[3] = {0x01, 0x01, 0x04};
        canWrite(hndW0, 0x202, data, 3, canMSG_STD);
    }
}

void CanInterface::steeringInfoCallback(std_msgs::Float32MultiArray msg)
{
    int8_t pMovementState = msg.data[0];
    this->steering_state = pMovementState;

    int16_t pPosition = msg.data[1]*2;
    int8_t pPositionBytes[3];
    this->actual_steering_angle = pPosition;
    intToBytes(pPosition, pPositionBytes);

    int16_t pTargetPosition = msg.data[2]*2;
    int8_t pTargetPositionBytes[3];
    this->target_steering_angle = pTargetPosition;
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
    uint8_t data[1] = {0x00};
    canWrite(hndW1, 0x183, data, 1, canMSG_STD);
}

void CanInterface::lapCounterCallback(std_msgs::Int16 msg)
{
    this->lap_counter = msg.data;
}

void CanInterface::conesCountCallback(sensor_msgs::PointCloud2 msg)
{
    this->cones_count_actual = msg.width;
}

void CanInterface::conesCountAllCallback(sensor_msgs::PointCloud2 msg)
{   
    if(this->cones_count_all<msg.width){
        if(msg.width < 500)
        {
            this->cones_count_all = msg.width;
        }
    }
}

void CanInterface::DL500Callback(const ros::TimerEvent&)
{
    std_msgs::Float32MultiArray x;

    int8_t data[8] = {motor_moment_target ,this->motor_moment_actual ,this->brake_hydr_target,this->brake_hydr_actual,
        this->target_steering_angle,this->actual_steering_angle,this->target_speed,this->actual_speed};

    x.data.push_back(motor_moment_target);
    x.data.push_back(this->motor_moment_actual);
    x.data.push_back(this->brake_hydr_target);
    x.data.push_back(this->brake_hydr_actual);
    x.data.push_back(this->target_steering_angle);
    x.data.push_back(this->actual_steering_angle);
    x.data.push_back(this->target_speed);
    x.data.push_back(this->actual_speed);

    this->DL500Pub.publish(x);

    canWrite(hndW0, 0x500, data, 8, canMSG_STD);
}

void CanInterface::DL501Callback(const ros::TimerEvent&)
{
    std_msgs::Float32MultiArray x;

    int16_t long_acc = IMUData.linear_acceleration.x*512;
    int8_t long_acc_bytes[2];
    intToBytes(long_acc, long_acc_bytes);
    int8_t long_acc_bytes_le[2] = {long_acc_bytes[1], long_acc_bytes[0]};
    x.data.push_back(IMUData.linear_acceleration.x);

    int16_t lat_acc = IMUData.linear_acceleration.y*512;
    int8_t lat_acc_bytes[2];
    intToBytes(lat_acc, lat_acc_bytes);
    int8_t lat_acc_bytes_le[2] = {lat_acc_bytes[1], lat_acc_bytes[0]};
    x.data.push_back(IMUData.linear_acceleration.y);

    int16_t yaw_rate = IMUData.angular_velocity.z*(180/M_PI)*128;
    int8_t yaw_rate_bytes[2];
    intToBytes(yaw_rate, yaw_rate_bytes);
    int8_t yaw_rate_bytes_le[2] = {yaw_rate_bytes[1], yaw_rate_bytes[0]};
    x.data.push_back(IMUData.angular_velocity.z);

    int8_t data[6];
    std::copy(long_acc_bytes_le, long_acc_bytes_le + 2, data + 4);
    std::copy(lat_acc_bytes_le, lat_acc_bytes_le + 2, data + 2);
    std::copy(yaw_rate_bytes_le, yaw_rate_bytes_le + 2, data);

    canWrite(hndW0, 0x501, data, 6, canMSG_STD);
    this->DL501Pub.publish(x);
}

void CanInterface::DL502Callback(const ros::TimerEvent&)
{   
    std_msgs::Float32MultiArray x;
    uint8_t data[5];
    data[4] = (((this->AMI_state <<2) | this->EBS_state)<<3) | this->AS_state;
    x.data.push_back(this->AMI_state);
    x.data.push_back(this->EBS_state);
    x.data.push_back(this->AS_state);
    data[3] = ((((((this->cones_count_actual & 0x01)<<4)|this->lap_counter)<<2)|this->service_brake_state)<<1)|steering_state;
    x.data.push_back(this->cones_count_actual);
    x.data.push_back(this->lap_counter);
    x.data.push_back(this->service_brake_state);
    x.data.push_back(this->steering_state);
    data[2] = (this->cones_count_all & 0x0001)|((this->cones_count_actual & 0xFE)>>1);
    x.data.push_back(this->cones_count_all);
    data[1] = (this->cones_count_all & 0x01FE)>>1;
    data[0] = (this->cones_count_all & 0xFE00)>>9;

    canWrite(hndW0, 0x502, data, 5, canMSG_STD);
    this->DL502Pub.publish(x);
}

void CanInterface::targetSpeedCallback(std_msgs::Int16 msg)
{
    this->target_speed = msg.data;
}

void CanInterface::pcTempCallback(const ros::TimerEvent&)
{
    this->getPcTemp();
    std_msgs::Float32 x;
    x.data = this->pc_temp;
    this->PCTempPub.publish(x);

    int8_t bytes[2];
    int16_t temp = this->pc_temp*100;
    intToBytes(temp, bytes);

    int16_t msg[3] = {0x01, bytes[0], bytes[1]};
    canWrite(hndW1, 0x183, msg, 3, canMSG_STD);
}

void CanInterface::brakeLightCallback(std_msgs::Int16 msg)
{
    uint8_t data[2] = {0x01, msg.data};
    canWrite(hndW1, 0x208, data, 2, canMSG_STD);
}

void CanInterface::getPcTemp()
{
    float temp = 0.0;
    FILE* fp = popen("sensors", "r");
    if (fp == NULL) {
        ROS_ERROR("Failed to run sensors command");
    }

    char path[1035];
    while (fgets(path, sizeof(path), fp) != NULL) {
        std::string line(path);
        if (line.find("Core 0:") != std::string::npos) { // Ajusta esto según tu salida de 'sensors'
            std::istringstream iss(line);
            std::string token;
            while (iss >> token) {
                if (token[0] == '+') {
                    token = token.substr(1);
                    token.pop_back();
                    temp = std::stof(token);
                    break;
                }
            }
            break;
        }
    }
    pclose(fp);
    this->pc_temp = temp;
}
