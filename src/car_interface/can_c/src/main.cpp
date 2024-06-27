#include <ros/ros.h>
#include "canInterface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_c");
    std::cout << "Powered by ADDA V3" << std::endl;
    // ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    CanInterface canInterface;

    return 0;
}