#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <common_msgs/CarState.h>
#include <iostream>
#include "PointXYZColorScore.h"

using namespace std;

class LidarHandle
{
public:
    string lidar_topic;
    string frame_id;
    string map_topic;
    string filtered_cloud_topic;
    string cones_marker_topic;
    string car_state_topic;

    float MAX_X_FOV;
    float MAX_Y_FOV;
    float MAX_Z_FOV;
    float H_FOV;

    float FRAMERATE;
    float yaw_rate;
    float vx;

    bool inverted;

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber car_state_sub;
    ros::Publisher map_pub;
    ros::Publisher filtered_cloud_pub;
    ros::Publisher markers_pub;

public:
    LidarHandle();
    void CarStateCallback(common_msgs::CarState);
    void callback(sensor_msgs::PointCloud2);
};
