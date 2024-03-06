#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>

using namespace std;

class LidarHandle {
    public:
        string lidar_topic;
        string frame_id;
        string map_topic;
        string filtered_cloud_topic;
        string cones_marker_topic;

        float MAX_X_FOV;
        float MAX_Y_FOV;
        float MAX_Z_FOV;
        float H_FOV;


        bool inverted;

        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher map_pub;
        ros::Publisher filtered_cloud_pub;
        ros::Publisher markers_pub;

    public:
        LidarHandle();
        void callback(sensor_msgs::PointCloud2);
};









