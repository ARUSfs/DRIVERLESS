#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>

using namespace std;

class LidarHandle {
    public:
        float MAX_X_FOV;
        float MAX_Y_FOV;
        float MAX_Z_FOV;
        float H_FOV;
        int N_SEGMENTS;

        string lidar_topic;
        string frame_id;
        bool inverted;

        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub2;
        ros::Publisher pub3;
        ros::Subscriber sub;

    public:
        LidarHandle();
        void callback(sensor_msgs::PointCloud2);
};









