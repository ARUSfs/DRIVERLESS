#include "ros/ros.h"
#include "ControlHandle.hpp"
#include <ros/console.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "PP_cpp");
	ControlHandle h;
	ros::spin();
    return 0;
}
