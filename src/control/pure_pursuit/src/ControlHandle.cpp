#include "ControlHandle.hpp"

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "common_msgs/Trajectory.h"
#include "common_msgs/Controls.h"
#include "common_msgs/CarState.h"
#include "geometry_msgs/Point.h"

#include <pcl/point_types.h>

#include <iostream>
#include <chrono>
#include <algorithm>

ControlHandle::ControlHandle(){

    nh.getParam("/pure_pursuit/TARGET_SPEED", TARGET_SPEED);
    nh.getParam("/pure_pursuit/KP", KP);
    nh.getParam("/pure_pursuit/KD", KD);
    nh.getParam("/pure_pursuit/KI", KI);
    nh.getParam("/pure_pursuit/previous_error", previous_error);
    nh.getParam("/pure_pursuit/integral", integral);


    velocity_sub = nh.subscribe("/car_state/state", 10, &ControlHandle::speed_callback, this);

    path_sub = nh.subscribe("/route", 1, &ControlHandle::path_callback, this);
    braking_sub = nh.subscribe("/braking", 1, &ControlHandle::braking_callback, this);
    speed_profile_sub = nh.subscribe("/speed_profile",1, &ControlHandle::update_target, this);
    sk_sub = nh.subscribe("controller/sk", 1, &ControlHandle::update_sk, this);


    control_publisher = nh.advertise<common_msgs::Controls>("/controls_pp", 1);
    pursuit_point_publisher = nh.advertise<geometry_msgs::Point>("pursuit_point",1);
    target_speed_pub = nh.advertise<std_msgs::Float32>("/target_speed", 1);
     
    
    previous_time = std::chrono::high_resolution_clock::now();


    publisher_timer = nh.createTimer(ros::Duration(0.01),
                                                &ControlHandle::control_timer_callback,
                                                this);

}

void ControlHandle::control_timer_callback(const ros::TimerEvent& event) {

    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delta_time = current_time - previous_time;
    previous_time = current_time;

    const float v_error = TARGET_SPEED - velocity;
    integral += v_error * delta_time.count();
    const float derivative = (v_error - previous_error)/delta_time.count(); 
    previous_error = v_error;
    const float accelerator_control = (v_error*KP)+(KI*integral)+(KD*derivative);

    const float angle = pPursuit.get_steering_angle();

    
    common_msgs::Controls msg;
    msg.accelerator = accelerator_control/230;
    msg.steering = angle;
    control_publisher.publish(msg);

    if(!braking){
        std_msgs::Float32 target_speed_msg;
        target_speed_msg.data = TARGET_SPEED;
        target_speed_pub.publish(target_speed_msg);
    }
    
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

void ControlHandle::braking_callback(const std_msgs::Bool braking_msg) {
    braking = braking_msg.data;
}

void ControlHandle::update_target(const std_msgs::Float32MultiArray msg){
    float dx =  0.1*std::max(velocity,3.0f);
    int index = 0;
    std::cout << dx << std::endl;
    std::cout << s.size() << std::endl;
    while (s.size()>index && s[index]<dx){
        index++;
    }
    if(msg.data.size()>index){
        TARGET_SPEED = msg.data[index];
    }
    std::cout << "target: " << TARGET_SPEED << std::endl;
}


void ControlHandle::update_sk(const common_msgs::Trajectory msg){
    if(msg.trajectory.size()>0){
        s.clear();
        k.clear();

        for(const geometry_msgs::Point &point : msg.trajectory){
            s.push_back(point.x);
            k.push_back(point.y);
        }
    } 
}


