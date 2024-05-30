#include "PurePursuit.hpp"

#include <cmath>

#include <pcl/point_types.h>
#include <pcl/common/norms.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <numeric>


using namespace std;
using namespace pcl;

void PurePursuit::update_path(const vector<pcl::PointXY> &new_path) {
    // Checking if path is empty since Delaunay sometimes returns empty paths
    // In this case just ignore it, the last path should be enough.
    if(new_path.empty()) {
        return;
    }

    path = new_path;
    path_updated = true;
}


PointXY PurePursuit::search_pursuit_point(const float look_ahead_distance, const pcl::PointXY car_position) {
    // TODO: Should check for empty path in class

    if(path.empty()){
        PointXY p;
        return p;
    }

    size_t closest_point_index = 0;
    float min_distance = numeric_limits<float>::infinity();

    if(path_updated) {
        distance_along_curve.clear();
    }

    for(size_t point_index = 0; point_index < path.size(); point_index++){
        const pcl::PointXY current_point = path[point_index];

        // Actually not the distance but L2 squared, since the actual distance
        // doesn't matter.
        const float dx = current_point.x - car_position.x;
        const float dy = current_point.y - car_position.y;
        const float current_distance = dx*dx + dy*dy; // Multiplying faster than pow

        if(current_distance < min_distance){
            min_distance = current_distance;
            closest_point_index = point_index;
        }

        // To avoid constantly calculating the length along the curve from
        // the beginning of it, just do it when the path has changed.
        // Doing it in this loop to avoid an extra loop.
        if(path_updated) {
            if(point_index == 0){
                distance_along_curve.push_back(0.0f);
            }
            else {
                const pcl::PointXY previous_point =  path[point_index-1];
                const float dist_dx = current_point.x - previous_point.x;
                const float dist_dy = current_point.y - previous_point.y;
                const float distance_from_previous = sqrt(dist_dx*dist_dx + dist_dy*dist_dy);
                distance_along_curve.push_back(distance_from_previous + distance_along_curve[distance_along_curve.size() - 1]);
            }
        }
    }

    const float closest_point_dist = distance_along_curve[closest_point_index];
    for(size_t point_index = closest_point_index; point_index < path.size(); point_index++) {
        const float cur_dist = distance_along_curve[point_index] - closest_point_dist;
        if(cur_dist > look_ahead_distance) {
            pursuit_index = point_index;
            return path[pursuit_index];
        }
    }

    return path[path.size() - 1];

}

float PurePursuit::get_steering_angle() {

    geometry_msgs::TransformStamped transform;
    pcl::PointXY car_position;
    try {
        transform = tfBuffer.lookupTransform("map", "rslidar" , ros::Time(0));
        car_position.x = transform.transform.translation.x;
        car_position.y = transform.transform.translation.y;
    } catch(tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return 0.0f;
    }

    pcl::PointXY pPoint = search_pursuit_point(3, car_position);
    //para evitar volantazos cuando no hay ruta
    if(pPoint.x*pPoint.x + pPoint.y*pPoint.y < 0.5){
        return 0.0f;
    }


    tf2::Quaternion quaternion;
    tf2::fromMsg(transform.transform.rotation, quaternion);
    tf2::Matrix3x3 rotationMatrix(quaternion);
    double roll, pitch, yaw;
    rotationMatrix.getRPY(roll, pitch, yaw);


    float alpha = atan2((pPoint.y - car_position.y),(pPoint.x - car_position.x))-yaw;

    float delta = atan2((2.0 * 1.5 * sin(alpha))/3,1.0);

    delta = max(-19.9,min(180*delta/M_PI,19.9));

    return delta;

}

PurePursuit::PurePursuit(){
    
}
