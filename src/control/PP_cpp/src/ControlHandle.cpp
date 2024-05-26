#include "ControlHandle.hpp"

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "common_msgs/Trajectory.h"
#include "common_msgs/Controls.h"
#include "geometry_msgs/Point.h"

#include <pcl/point_types.h>


ControlHandle::ControlHandle() :
{

    velocity_sub = nh.subscribe("/motor_speed", 10, &ControlHandle::motor_speed_callback, this);

    path_sub = nh.subscribe("/control_pure_pursuit/route_topic", 1, &ControlHandle::path_callback, this);

    control_publisher = nh.advertise<common_msgs::Controls>("/controls", 1);
    ros::Timer publisher_timer = nh.createTimer(ros::Duration(0.01),
                                                &ControlHandle::control_timer_callback,
                                                this);
}

void ControlHandle::control_timer_callback(const ros::TimerEvent& event) {
    // TODO PONER CON PARAMETROS POR DIOS ESTO ES PARA PROBAR TODO
    const float TARGET_VEL = 3;
    const float Kp = 1;

    const float v_error = TARGET_VEL - velocity;
    const float accelerator_control = v_error*Kp;

    const angle = pPursuit.get_steering_angle()


}

void ControlHandle::motor_speed_callback(const std_msgs::Float32 new_vel) {
    velocity = 0.9*velocity + 0.1*new_vel.data;
}

void ControlHandle::path_callback(const common_msgs::Trajectory path) {

    std::vector<pcl::PointXY> new_path;

    for(const geometry_msgs::Point &point : path.trajectory){
        const pcl::PointXY tmp_point = {point.x, point.y};
        new_path.push_back(tmp_point);
    }


    pPursuit.update_path(new_path);
}
