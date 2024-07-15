#include "ControlHandle.hpp"

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "common_msgs/Trajectory.h"
#include "common_msgs/Controls.h"
#include "common_msgs/CarState.h"
#include "geometry_msgs/Point.h"

#include <pcl/point_types.h>


ControlHandle::ControlHandle(){

    nh.getParam("/pure_pursuit/TARGET_SPEED", TARGET_SPEED);
    nh.getParam("/pure_pursuit/KP", KP);
    nh.getParam("/pure_pursuit/KI", KI);
    nh.getParam("/pure_pursuit/KD", KD);

    velocity_sub = nh.subscribe("/car_state/state", 10, &ControlHandle::speed_callback, this);

    path_sub = nh.subscribe("/route", 1, &ControlHandle::path_callback, this);

    control_publisher = nh.advertise<common_msgs::Controls>("/controls_pp", 1);
    pursuit_point_publisher = nh.advertise<geometry_msgs::Point>("pursuit_point",1);
    publisher_timer = nh.createTimer(ros::Duration(0.01),
                                                &ControlHandle::control_timer_callback,
                                                this);

}

void ControlHandle::control_timer_callback(const ros::TimerEvent& event) {
    // TODO PONER CON PARAMETROS POR DIOS ESTO ES PARA PROBAR TODO

    const float error = TARGET_SPEED - velocity;
    const float dt = ros::Time::now().toSec() - prev_t;
    integral += error*dt;
    const float derivative = (error-prev_err)/dt;

    prev_t = ros::Time::now().toSec();
    prev_err = error;

    const float accelerator_control = error*KP + KI*integral + KD*derivative;
    
    const float angle = pPursuit.get_steering_angle();

    
    common_msgs::Controls msg;
    msg.accelerator = accelerator_control;
    msg.steering = angle;
    control_publisher.publish(msg);

    if(!pPursuit.path.empty()){
        geometry_msgs::Point p;
        p.x = pPursuit.path[pPursuit.pursuit_index].x;
        p.y = pPursuit.path[pPursuit.pursuit_index].y;
        pursuit_point_publisher.publish(p);
    }

}

void ControlHandle::speed_callback(const common_msgs::CarState new_state) {
    velocity = sqrt(new_state.vx*new_state.vx + new_state.vy*new_state.vy);
}

void ControlHandle::path_callback(const common_msgs::Trajectory path) {

    std::vector<pcl::PointXY> new_path;

    for(const geometry_msgs::Point &point : path.trajectory){
        const pcl::PointXY tmp_point = {point.x, point.y};
        new_path.push_back(tmp_point);
    }


    pPursuit.update_path(new_path);
}
