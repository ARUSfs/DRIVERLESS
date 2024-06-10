#include <ros/ros.h>
#include "canInterface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_c");
    std::cout << "shgshgfs" << std::endl;
    CanInterface canInterface;
    ros::spin();
    return 0;
}