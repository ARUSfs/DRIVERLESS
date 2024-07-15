#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "common_msgs/Trajectory.h"
#include "common_msgs/CarState.h"

#include "PurePursuit.hpp"

class ControlHandle {
    private:
        float velocity = 0;
        float TARGET_SPEED;
        float KP;
        float KD;
        float KI;
        float previous_error;
        float integral;
        std::chrono::time_point<std::chrono::high_resolution_clock> previous_time;

        ros::NodeHandle nh;
        ros::Publisher control_publisher;
        ros::Publisher pursuit_point_publisher;
        ros::Subscriber velocity_sub;
        ros::Subscriber path_sub;
        ros::Timer publisher_timer;


        PurePursuit pPursuit;

        void speed_callback(const common_msgs::CarState);
        void path_callback(const common_msgs::Trajectory);
        void control_timer_callback(const ros::TimerEvent&);

    public:
        ControlHandle();

};

