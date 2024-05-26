#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "common_msgs/Trajectory.h"

#include "PurePursuit.hpp"

class ControlHandle {
    private:
        float velocity = 0;

        ros::NodeHandle nh;
        ros::Publisher control_publisher;
        ros::Subscriber velocity_sub;
        ros::Subscriber path_sub;


        PurePursuit pPursuit;

        void motor_speed_callback(const std_msgs::Float32);
        void path_callback(const common_msgs::Trajectory);
        void control_timer_callback(const ros::TimerEvent&);

    public:
        ControlHandle();

};
