#include "ros/ros.h"

#include "lidar_handle.hpp"


int main(int argc, char **argv){

    ros::init(argc,argv,"new_lidar_perception");

    LidarHandle lidarHandle;

    ros::spin();

    return 0;
}