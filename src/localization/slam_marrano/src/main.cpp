#include "ros/ros.h"
#include "icp_slam.hpp"
#include <ros/console.h>
#include <pcl/registration/icp.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "icp_odometry");
	ICP_handle h;
	ros::spin();

	return 0;
}
