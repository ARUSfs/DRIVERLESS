#include <canlib.h>
#include <stdio.h>
#include <thread>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/PointCloud2.h"
#include <common_msgs/Controls.h>

using namespace std;

class CanInterface
{

public:
    CanInterface();
    
    static void check_can(canStatus stat);
private :

    void pubHeartBeat(const ros::TimerEvent&);
    void controlsCallback(common_msgs::Controls);
    void brakeLightCallback(std_msgs::Int16);
    void steeringInfoCallback(std_msgs::Float32MultiArray);
    void ASStatusCallback(std_msgs::Int16);
    void lapCounterCallback(std_msgs::Int16);
    void conesCountCallback(sensor_msgs::PointCloud2);
    void conesCountAllCallback(sensor_msgs::PointCloud2);
    void targetSpeedCallback(std_msgs::Float32);
    void DL500Callback(const ros::TimerEvent&);
    void DL501Callback(const ros::TimerEvent&);
    void DL502Callback(const ros::TimerEvent&);
    void pcTempCallback(const ros::TimerEvent&);

    void readCan1();
    void readCan0();

    canHandle hndW0;
    canHandle hndW1;

    ros::NodeHandle nh;

    ros::Publisher extensometerPub;
    ros::Publisher ASStatusPub;
    ros::Publisher IMUPub;
    ros::Publisher GPSPub;
    ros::Publisher GPSSpeedPub;
    ros::Publisher motorSpeedPub;
    ros::Publisher steeringAnglePub;
    ros::Publisher RESRangePub;
    ros::Publisher PCTempPub;
    ros::Publisher DL500Pub;
    ros::Publisher DL501Pub;
    ros::Publisher DL502Pub;

    float pc_temp;
    void getPcTemp();

    uint8_t actual_speed;
    uint8_t target_speed;
    int8_t actual_steering_angle;
    int8_t target_steering_angle;
    uint8_t brake_hydr_actual;
    uint8_t brake_hydr_target;
    uint8_t pneumatic_press;
    int8_t motor_moment_actual;
    int8_t motor_moment_target;

    sensor_msgs::Imu IMUData;
    
    uint8_t AS_state;
    uint8_t EBS_state;
    uint8_t AMI_state;
    bool steering_state;
    uint8_t service_brake_state;
    uint8_t lap_counter;
    uint8_t cones_count_actual;
    uint16_t cones_count_all;

    void parseInvSpeed(unsigned char []);
    void parseMission(unsigned char []);
    void parseASStatus(unsigned char []);
    void parseAcc(unsigned char []);
    void parseEulerAngles(unsigned char []);
    void parseAngularVelocity(unsigned char []);
    void parseGPS(unsigned char []);
    void parseGPSVel(unsigned char []);
    void parseSteeringAngle(unsigned char []);
    void parseRES(unsigned char []);
    void parseBrakeHydr(unsigned char []);
    void parsePneumatic(unsigned char []);
    void initialize_timer();


    ros::Subscriber controlsSub;
    ros::Subscriber steeringInfoSub;
    ros::Subscriber ASStatusSub;
    ros::Subscriber lapCounterSub;
    ros::Subscriber conesCountSub;
    ros::Subscriber conesCountAllSub;
    ros::Subscriber targetSpeedSub;
    ros::Subscriber brakeLightSub;

    ros::Timer pcTempTimer;
    ros::Timer heartBeatTimer;
    ros::Timer DL500Timer;
    ros::Timer DL501Timer;
    ros::Timer DL502Timer;

    std::thread thread_0;
    std::thread thread_1;    
};
