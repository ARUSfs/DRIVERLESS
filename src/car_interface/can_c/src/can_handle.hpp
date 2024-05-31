#include <canlib.h>
#include <stdio.h>
#include <thread>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <common_msgs/Controls.h>

using namespace std;

class CanInterface
{

public:
    CanInterface();
    void pubIMU(const ros::TimerEvent&);
    void pubHeartBeat(const ros::TimerEvent&);
    void controlsCallback(common_msgs::Controls);
    void steeringInfoCallback(std_msgs::Int32MultiArray);
    void ASStatusCallback(std_msgs::Int16);
    void readCan1();
    void readCan0();

    sensor_msgs::Imu IMUData;
    canHandle hndW0;
    canHandle hndW1;

private :
    void parseInvSpeed(unsigned char []);
    void parseASStatus(unsigned char []);
    void parseAcc(unsigned char []);
    void parseEulerAngles(unsigned char []);
    void parseAngularVelocity(unsigned char []);
    void parseGPS(unsigned char []);
    void parseGPSVel(unsigned char []);
    void parseSteeringAngle(unsigned char []);
    void parseRearWheelSpeed(unsigned char []);
    void parseFrontWheelSpeed(unsigned char []);

    ros::NodeHandle nh;

    ros::Subscriber controlsSub;
    ros::Subscriber steeringInfoSub;
    ros::Subscriber ASStatusSub;

    ros::Publisher extensometerPub;
    ros::Publisher ASStatusPub;
    ros::Publisher IMUPub;
    ros::Publisher GPSPub;
    ros::Publisher GPSSpeedPub;
    ros::Publisher motorSpeedPub;
    ros::Publisher steeringAnglePub;
    ros::Publisher rearWheelSpeedPub;
    ros::Publisher frontWheelSpeedPub;

    ros::Timer heartBeatTimer;
    ros::Timer IMUTimer;

    std::thread thread_0;
};
