#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
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
        bool braking = false;
        std::vector<float> s;
        std::vector<float> k;

        std::chrono::time_point<std::chrono::high_resolution_clock> previous_time;

        ros::NodeHandle nh;
        ros::Publisher control_publisher;
        ros::Publisher pursuit_point_publisher;
        ros::Publisher target_speed_pub;
        ros::Subscriber velocity_sub;
        ros::Subscriber braking_sub;
        ros::Subscriber path_sub;
        ros::Subscriber sk_sub;
        ros::Subscriber speed_profile_sub;
        ros::Timer publisher_timer;


        PurePursuit pPursuit;

        void speed_callback(const common_msgs::CarState);
        void path_callback(const common_msgs::Trajectory);
        void control_timer_callback(const ros::TimerEvent&);
        void braking_callback(const std_msgs::Bool);
        void update_target(const std_msgs::Float32MultiArray);
        void update_sk(const common_msgs::Trajectory);

    public:
        ControlHandle();

};

