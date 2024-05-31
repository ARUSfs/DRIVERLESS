#include <ros/ros.h>
#include "can_handle.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_c");
    CanHandle canHandle;
    ros::spin();
    return 0;
}